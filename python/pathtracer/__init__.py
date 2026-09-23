"""Headless access to the PATHTRACER renderer, returning AOVs as numpy arrays.

    >>> from pathtracer import Renderer
    >>> renderer = Renderer("scenes/cornell.json")
    >>> frame = renderer.render(width=256, height=256, samples=64, aovs=("beauty", "depth", "normal"))
    >>> frame["beauty"].shape
    (256, 256, 3)

Values are scene-referred linear, unclamped, with no display transform applied: what a model should train on, not
what a monitor should show. Use ``render_beauty --out`` for a display-encoded picture.

Arrays are C-contiguous float32, so ``torch.from_numpy(frame["beauty"])`` shares memory with no copy, leaving one
explicit ``.to(device)``.
"""

from __future__ import annotations

import ctypes
from collections.abc import Iterable, Mapping
from dataclasses import dataclass

import numpy as np

from . import _ffi

__all__ = ["AOVS", "Camera", "Renderer", "aov_channels", "aov_needs_samples"]

_LIB = _ffi.load_library()

#: Every AOV the renderer can produce, in the renderer's own order. Read from the library rather than restated here,
#: so Python cannot hold a stale copy of a list the C++ side owns.
AOVS: tuple[str, ...] = tuple(
    _LIB.pt_aov_name(index).decode() for index in range(_LIB.pt_aov_count())
)


def _aov_id(name: str) -> int:
    # int() rather than a cast: ctypes types every foreign return as Any, and the restype declared in _ffi is the
    # only thing that makes this an int at all, so converting is the honest way to re-enter the typed world.
    identifier = int(_LIB.pt_aov_id(name.encode()))
    if identifier < 0:
        raise ValueError(f"unknown AOV {name!r}; known AOVs are: {', '.join(AOVS)}")
    return identifier


def aov_channels(name: str) -> int:
    """Channels this AOV carries: 1 for a depth or filter response, 2 for UV, 3 for radiance and vectors."""
    return int(_LIB.pt_aov_channels(_aov_id(name)))


def aov_needs_samples(name: str) -> bool:
    """Whether ``samples`` affects this AOV.

    False for the 14 primary-hit AOVs the rasterizer scan-converts in a single pass; those are essentially free and
    converge immediately, so raising ``samples`` for them only wastes time.
    """
    return bool(_LIB.pt_aov_needs_samples(_aov_id(name)))


@dataclass(frozen=True)
class Camera:
    """Pose, lens and exposure.

    ``aperture``, ``shutter_seconds`` and ``iso`` set the photographic exposure value only. The camera is a pinhole:
    none of the three produces depth of field, and ``aperture`` is not a lens radius.

    Immutable, so an override is a ``dataclasses.replace`` of ``Renderer.default_camera`` rather than a mutation
    that could leak between renders.
    """

    position: tuple[float, float, float]
    yaw_degrees: float
    pitch_degrees: float
    film_back_mm: tuple[float, float]
    focal_length_mm: float
    near_clip: float
    far_clip: float
    aperture: float
    shutter_seconds: float
    iso: float

    @classmethod
    def _from_struct(cls, struct: _ffi.PtCamera) -> Camera:
        return cls(
            position=(struct.position[0], struct.position[1], struct.position[2]),
            yaw_degrees=struct.yaw_degrees,
            pitch_degrees=struct.pitch_degrees,
            film_back_mm=(struct.film_back_mm[0], struct.film_back_mm[1]),
            focal_length_mm=struct.focal_length_mm,
            near_clip=struct.near_clip,
            far_clip=struct.far_clip,
            aperture=struct.aperture,
            shutter_seconds=struct.shutter_seconds,
            iso=struct.iso,
        )

    def _to_struct(self) -> _ffi.PtCamera:
        struct = _ffi.PtCamera()
        struct.position = (ctypes.c_float * 3)(*self.position)
        struct.yaw_degrees = self.yaw_degrees
        struct.pitch_degrees = self.pitch_degrees
        struct.film_back_mm = (ctypes.c_float * 2)(*self.film_back_mm)
        struct.focal_length_mm = self.focal_length_mm
        struct.near_clip = self.near_clip
        struct.far_clip = self.far_clip
        struct.aperture = self.aperture
        struct.shutter_seconds = self.shutter_seconds
        struct.iso = self.iso
        return struct


class Renderer:
    """One loaded scene, rendered as many times as you like.

    Loading builds the BVH, the environment-map CDFs and the thread pool, so construct once and call ``render``
    repeatedly rather than reconstructing per frame.
    """

    def __init__(self, scene_path: str, asset_root: str | None = None) -> None:
        error = _ffi.make_error_buffer()
        handle = _LIB.pt_renderer_open(
            asset_root.encode() if asset_root is not None else None,
            scene_path.encode(),
            error,
            len(error),
        )
        if not handle:
            raise RuntimeError(error.value.decode())
        self._handle = handle

    def close(self) -> None:
        """Releases the scene, its BVH and its thread pool. Idempotent."""
        handle, self._handle = getattr(self, "_handle", None), None
        if handle:
            _LIB.pt_renderer_close(handle)

    def __del__(self) -> None:
        self.close()

    def __enter__(self) -> Renderer:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @property
    def default_camera(self) -> Camera:
        """profile.json's authored camera, the starting point for an override."""
        struct = _ffi.PtCamera()
        _LIB.pt_renderer_default_camera(self._handle, ctypes.byref(struct))
        return Camera._from_struct(struct)

    @property
    def default_resolution(self) -> tuple[int, int]:
        return (_LIB.pt_renderer_default_width(self._handle), _LIB.pt_renderer_default_height(self._handle))

    def render(
        self,
        *,
        aovs: Iterable[str] = ("beauty",),
        width: int | None = None,
        height: int | None = None,
        samples: int = 1,
        seed: int = 1,
        camera: Camera | None = None,
    ) -> Mapping[str, np.ndarray]:
        """Renders the requested AOVs and returns them keyed by the names given.

        Each producer runs at most once per call, so asking for several AOVs together costs far less than asking for
        them separately: the 10 path-traced lanes share one sample set, the 14 rasterizer lanes share one
        scan-conversion, and the Beauty filters share the one accumulated Beauty.

        ``samples`` is the number of one-sample passes averaged. The sampler's scramble is fixed by ``seed`` and its
        sequence index advances per pass, so the same arguments always reproduce the same floats exactly.

        Returns arrays of shape ``(height, width, channels)``, float32, row 0 at the top.
        """
        names = tuple(aovs)
        if not names:
            raise ValueError("at least one AOV is required")
        if samples < 1:
            raise ValueError(f"samples must be at least 1, got {samples}")

        default_width, default_height = self.default_resolution
        width = default_width if width is None else width
        height = default_height if height is None else height
        if width < 1 or height < 1:
            raise ValueError(f"resolution must be positive, got {width}x{height}")

        identifiers = [_aov_id(name) for name in names]
        # Allocated here and handed down as bare pointers: numpy owns every byte, so nothing has to be freed across
        # the ABI and the arrays outlive the call without a copy.
        buffers = [
            np.empty((height, width, _LIB.pt_aov_channels(identifier)), dtype=np.float32)
            for identifier in identifiers
        ]
        pointers = (ctypes.POINTER(ctypes.c_float) * len(buffers))(
            *(buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) for buffer in buffers)
        )

        request = _ffi.PtRenderRequest()
        request.camera = (camera if camera is not None else self.default_camera)._to_struct()
        request.width = width
        request.height = height
        request.samples = samples
        request.seed = seed
        request.aovs = (ctypes.c_int * len(identifiers))(*identifiers)
        request.aov_count = len(identifiers)

        error = _ffi.make_error_buffer()
        if _LIB.pt_render(self._handle, ctypes.byref(request), pointers, error, len(error)) != 0:
            raise RuntimeError(error.value.decode())
        return dict(zip(names, buffers, strict=True))
