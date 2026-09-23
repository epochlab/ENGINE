"""Contract tests for the Python binding.

The strongest check here is ``test_beauty_matches_render_beauty_cli``: the renderer is deterministic by construction
(``render_beauty --assert-deterministic`` gates it), so Python's Beauty must be EXACTLY equal to the CLI's EXR at the
same scene, resolution, seed and pass count. Any tolerance there would hide the bugs worth catching.
"""

from __future__ import annotations

import dataclasses
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from pathtracer import AOVS, Renderer, aov_channels, aov_needs_samples

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE = "scenes/cornell.json"


@pytest.fixture(scope="module")
def renderer() -> Renderer:
    with Renderer(SCENE) as instance:
        yield instance


def test_aov_table_is_populated() -> None:
    assert len(AOVS) == 27
    for name in ("Beauty", "Depth", "Normal", "Sobel", "Luminance", "Gabor", "HSV"):
        assert name in AOVS


def test_aov_names_are_separator_insensitive() -> None:
    assert aov_channels("bounce-count") == aov_channels("Bounce Count") == 1
    assert aov_channels("indirect_specular") == 3


def test_shapes_and_dtypes_match_the_declared_channels(renderer: Renderer) -> None:
    names = ("beauty", "depth", "normal", "uv")
    frame = renderer.render(aovs=names, width=32, height=24, samples=2)
    for name in names:
        array = frame[name]
        assert array.shape == (24, 32, aov_channels(name))
        assert array.dtype == np.float32
        assert array.flags["C_CONTIGUOUS"], "torch.from_numpy needs a contiguous buffer to share memory"


def test_same_seed_reproduces_exactly(renderer: Renderer) -> None:
    first = renderer.render(aovs=("beauty",), width=32, height=24, samples=4, seed=3)["beauty"]
    second = renderer.render(aovs=("beauty",), width=32, height=24, samples=4, seed=3)["beauty"]
    assert np.array_equal(first, second)


def test_different_seeds_differ(renderer: Renderer) -> None:
    first = renderer.render(aovs=("beauty",), width=32, height=24, samples=4, seed=3)["beauty"]
    second = renderer.render(aovs=("beauty",), width=32, height=24, samples=4, seed=9)["beauty"]
    assert not np.array_equal(first, second)


def test_depth_is_positive_inside_the_box_and_far_on_the_background(renderer: Renderer) -> None:
    frame = renderer.render(aovs=("depth", "alpha"), width=64, height=48)
    depth, alpha = frame["depth"][..., 0], frame["alpha"][..., 0]
    assert np.isfinite(depth).all()
    hit = alpha > 0.0
    assert hit.any(), "the default camera should see geometry"
    assert (depth[hit] > 0.0).all()
    # The Cornell box encloses the camera, so every primary ray hits something well inside the far clip.
    assert depth[hit].max() < renderer.default_camera.far_clip


def test_normals_are_unit_length_where_geometry_was_hit(renderer: Renderer) -> None:
    frame = renderer.render(aovs=("normal", "alpha"), width=64, height=48)
    hit = frame["alpha"][..., 0] > 0.0
    lengths = np.linalg.norm(frame["normal"][hit], axis=-1)
    assert np.allclose(lengths, 1.0, atol=1e-5)


def test_rasterizer_aovs_need_no_samples() -> None:
    assert aov_needs_samples("beauty")
    assert aov_needs_samples("sobel"), "filters read Beauty, so they do need light transport"
    assert not aov_needs_samples("depth")
    assert not aov_needs_samples("normal")


def test_gbuffer_only_request_skips_the_path_tracer(renderer: Renderer) -> None:
    """A depth+normal request must not path-trace, which is visible as an order-of-magnitude time difference."""
    size = {"width": 96, "height": 96}

    start = time.perf_counter()
    renderer.render(aovs=("depth", "normal"), samples=64, **size)
    gbuffer_seconds = time.perf_counter() - start

    start = time.perf_counter()
    renderer.render(aovs=("depth", "normal", "beauty"), samples=64, **size)
    with_beauty_seconds = time.perf_counter() - start

    assert gbuffer_seconds * 10 < with_beauty_seconds, (
        f"G-buffer-only took {gbuffer_seconds:.4f}s against {with_beauty_seconds:.4f}s with Beauty; "
        "the path tracer appears to be running for a request that needs no light transport"
    )


def test_every_aov_renders(renderer: Renderer) -> None:
    for name in AOVS:
        array = renderer.render(aovs=(name,), width=32, height=32, samples=2)[name]
        assert array.shape == (32, 32, aov_channels(name))
        assert np.isfinite(array).all(), f"{name} produced a non-finite value"


def test_multi_aov_request_matches_single_requests(renderer: Renderer) -> None:
    names = ("beauty", "depth", "normal", "sobel")
    size = {"width": 32, "height": 32, "samples": 4, "seed": 5}
    together = renderer.render(aovs=names, **size)
    for name in names:
        alone = renderer.render(aovs=(name,), **size)[name]
        assert np.array_equal(alone, together[name]), f"{name} changed when requested alongside others"


def test_camera_override_changes_the_image(renderer: Renderer) -> None:
    default = renderer.default_camera
    moved = dataclasses.replace(default, yaw_degrees=default.yaw_degrees + 25.0)
    size = {"width": 32, "height": 32, "samples": 2}
    assert not np.array_equal(
        renderer.render(aovs=("depth",), **size)["depth"],
        renderer.render(aovs=("depth",), camera=moved, **size)["depth"],
    )


def test_unknown_aov_names_the_offender(renderer: Renderer) -> None:
    with pytest.raises(ValueError, match="nonsense"):
        renderer.render(aovs=("nonsense",), width=8, height=8)


def test_invalid_arguments_are_rejected(renderer: Renderer) -> None:
    with pytest.raises(ValueError):
        renderer.render(aovs=(), width=8, height=8)
    with pytest.raises(ValueError):
        renderer.render(aovs=("beauty",), width=8, height=8, samples=0)
    with pytest.raises(ValueError):
        renderer.render(aovs=("beauty",), width=0, height=8)


def test_unknown_scene_raises() -> None:
    with pytest.raises(RuntimeError):
        Renderer("scenes/does_not_exist.json")


def test_beauty_matches_render_beauty_cli(renderer: Renderer, tmp_path: Path) -> None:
    """Python's Beauty must be bit-identical to the CLI's, which is what proves the two paths are one renderer."""
    openexr = pytest.importorskip("OpenEXR", reason="reading the reference EXR needs the OpenEXR module")
    imath = pytest.importorskip("Imath")

    binary = REPO_ROOT / "build" / "render_beauty"
    if not binary.exists():
        pytest.skip("render_beauty is not built")

    reference = tmp_path / "cli.exr"
    subprocess.run(
        [str(binary), "--scene", SCENE, "--width", "64", "--height", "64",
         "--passes", "8", "--seed", "1", "--aov", "beauty",
         # --out is mandatory; the PNG is display-encoded and discarded, the EXR is the linear reference.
         "--out", str(tmp_path / "cli.png"), "--out-exr", str(reference)],
        check=True, capture_output=True,
    )

    exr = openexr.InputFile(str(reference))
    pixel = imath.PixelType(imath.PixelType.FLOAT)
    channels = [np.frombuffer(exr.channel(c, pixel), dtype=np.float32).reshape(64, 64) for c in ("R", "G", "B")]
    expected = np.stack(channels, axis=-1)

    actual = renderer.render(aovs=("beauty",), width=64, height=64, samples=8, seed=1)["beauty"]
    assert np.array_equal(actual, expected)
