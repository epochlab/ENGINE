#include "pathtracer/platform/display_link.h"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>

#include <pthread/qos.h>

#import <AppKit/AppKit.h>
#import <QuartzCore/QuartzCore.h>

#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include "pathtracer/platform/window.h"

// Times are CACurrentMediaTime seconds, the timebase CADisplayLink reports in.
struct pathtracer::platform::DisplayLink::State {
    std::mutex mutex;
    std::condition_variable vblank;
    std::uint64_t ticks = 0;     // vblanks delivered by the link
    std::uint64_t consumed = 0;  // ticks as of the last waitForNextVblank return
    double nextVblank = 0.0;
    double period = 0.0;
    bool paused = false;  // a vblank went a full period undelivered: free-running on the period grid until the link resumes
    CADisplayLink* link = nil;
    CFRunLoopRef runLoop = nullptr;
    std::thread thread;
};

@interface EngineDisplayLinkTarget : NSObject
- (instancetype)initWithState:(pathtracer::platform::DisplayLink::State*)state;
- (void)vblank:(CADisplayLink*)link;
@end

@implementation EngineDisplayLinkTarget {
    pathtracer::platform::DisplayLink::State* state_;
}

- (instancetype)initWithState:(pathtracer::platform::DisplayLink::State*)state {
    if ((self = [super init])) {
        state_ = state;
    }
    return self;
}

- (void)vblank:(CADisplayLink*)link {
    {
        const std::lock_guard lock(state_->mutex);
        ++state_->ticks;
        state_->nextVblank = link.targetTimestamp;
        state_->period = link.targetTimestamp - link.timestamp;
    }
    state_->vblank.notify_one();
}

@end

namespace pathtracer::platform {

DisplayLink::DisplayLink(const Window& window) : state_(std::make_unique<State>()) {
    NSView* view = glfwGetCocoaView(window.nativeHandle());
    NSScreen* screen = view.window.screen != nil ? view.window.screen : NSScreen.mainScreen;
    state_->period = screen.minimumRefreshInterval;
    state_->nextVblank = CACurrentMediaTime() + state_->period;
    // The link retains its target; the target borrows state_, which outlives the link thread (joined in the destructor).
    state_->link = [view displayLinkWithTarget:[[EngineDisplayLinkTarget alloc] initWithState:state_.get()]
                                      selector:@selector(vblank:)];
    // A run loop of its own, so ticks arrive while the render thread is blocked in waitForNextVblank rather than only inside glfwPollEvents.
    std::promise<CFRunLoopRef> started;
    std::future<CFRunLoopRef> runLoop = started.get_future();
    state_->thread = std::thread([state = state_.get(), &started] {
        // User-interactive, as for any display-timed work: at default QoS the trace threads delay tick delivery, and with it the render thread's wake, by milliseconds.
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
        @autoreleasepool {
            [state->link addToRunLoop:NSRunLoop.currentRunLoop forMode:NSDefaultRunLoopMode];
            started.set_value(CFRunLoopGetCurrent());
            CFRunLoopRun();
        }
    });
    state_->runLoop = runLoop.get();
}

DisplayLink::~DisplayLink() {
    CADisplayLink* link = state_->link;
    // Invalidated on its own run loop's thread, so no tick can be mid-delivery when state_ is freed.
    CFRunLoopPerformBlock(state_->runLoop, kCFRunLoopDefaultMode, ^{
      [link invalidate];
      CFRunLoopStop(CFRunLoopGetCurrent());
    });
    CFRunLoopWakeUp(state_->runLoop);
    state_->thread.join();
}

void DisplayLink::waitForNextVblank() {
    State& s = *state_;
    std::unique_lock lock(s.mutex);
    if (s.ticks == s.consumed) {
        const double now = CACurrentMediaTime();
        // Unpaused: the pending vblank counts as missed once a full period passes without it. Paused: the next point on the grid it left off.
        const double deadline = s.paused ? s.nextVblank + std::ceil((now - s.nextVblank) / s.period) * s.period
                                         : s.nextVblank + s.period;
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(deadline - now));
        if (!s.vblank.wait_until(lock, until, [&s] { return s.ticks != s.consumed; })) {
            s.paused = true;
            s.nextVblank = deadline;
            return;
        }
    }
    s.paused = false;
    s.consumed = s.ticks;
}

double DisplayLink::refreshPeriodSeconds() const {
    const std::lock_guard lock(state_->mutex);
    return state_->period;
}

}  // namespace pathtracer::platform
