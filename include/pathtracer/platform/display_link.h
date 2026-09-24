#pragma once

#include <memory>

namespace pathtracer::platform {

class Window;

// Paces the render loop to the vblank of the display the window is on, via -[NSView displayLinkWithTarget:selector:]
// (macOS 14+), which follows the window across displays. Replaces NSGL's swap interval: on current macOS that lets
// two swaps through per refresh while the window is visible, and GLFW substitutes a fixed 60 Hz usleep while it is not.
class DisplayLink {
public:
    explicit DisplayLink(const Window& window);
    ~DisplayLink();

    DisplayLink(const DisplayLink&) = delete;
    DisplayLink& operator=(const DisplayLink&) = delete;
    DisplayLink(DisplayLink&&) = delete;
    DisplayLink& operator=(DisplayLink&&) = delete;

    // Returns at the next vblank, or at once if one has passed since the last return. While the link is paused, a
    // minimised window, it free-runs on the display's period.
    void waitForNextVblank();

    // The display's refresh period, targetTimestamp - timestamp of the latest vblank, seeded from the screen's
    // minimumRefreshInterval until the first one arrives.
    [[nodiscard]] double refreshPeriodSeconds() const;

    struct State;

private:
    std::unique_ptr<State> state_;
};

}  // namespace pathtracer::platform
