#pragma once

#include <memory>

namespace engine::platform {

class Window;

// Paces the render loop to the vblank of the display the window is on, via -[NSView displayLinkWithTarget:selector:] (macOS 14+), which follows the window across displays.
// Replaces NSGL's swap interval: on current macOS it lets two swaps through per refresh while the window is visible, and GLFW substitutes a fixed 60 Hz usleep while it is occluded.
class DisplayLink {
public:
    explicit DisplayLink(const Window& window);
    ~DisplayLink();

    DisplayLink(const DisplayLink&) = delete;
    DisplayLink& operator=(const DisplayLink&) = delete;
    DisplayLink(DisplayLink&&) = delete;
    DisplayLink& operator=(DisplayLink&&) = delete;

    // Returns at the next vblank, or at once if one passed since the last return; while the link is paused (minimised window) it free-runs on the display's period grid instead of blocking.
    void waitForNextVblank();

    // The display's refresh period, targetTimestamp - timestamp of the latest vblank; seeded from the screen's minimumRefreshInterval until the first one.
    [[nodiscard]] double refreshPeriodSeconds() const;

    struct State;

private:
    std::unique_ptr<State> state_;
};

}  // namespace engine::platform
