#pragma once

#include <memory>

namespace pathtracer::platform {

class Window;

// Paces the render loop to the window's display vblank via displayLinkWithTarget: (macOS 14+), which NSGL's swap interval does not do.
class DisplayLink {
public:
    explicit DisplayLink(const Window& window);
    ~DisplayLink();

    DisplayLink(const DisplayLink&) = delete;
    DisplayLink& operator=(const DisplayLink&) = delete;
    DisplayLink(DisplayLink&&) = delete;
    DisplayLink& operator=(DisplayLink&&) = delete;

    // Returns at the next vblank, or at once if one has passed. While paused (a minimised window) it free-runs on the display's period.
    void waitForNextVblank();

    // The display's refresh period from the latest vblank, seeded from the screen's minimumRefreshInterval until the first one arrives.
    [[nodiscard]] double refreshPeriodSeconds() const;

    struct State;

private:
    std::unique_ptr<State> state_;
};

}  // namespace pathtracer::platform
