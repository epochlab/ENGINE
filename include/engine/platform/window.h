#pragma once

#include <functional>
#include <string>
#include <utility>

// Forward-declared rather than #include <GLFW/glfw3.h>: a pointer to an incomplete type is sufficient here, and keeping GLFW out of this public header means consumers that only need shouldClose()/pollEvents()/etc. don't drag GLFW/GL declarations in with them.
struct GLFWwindow;

namespace engine::platform {

// Owns a single GLFWwindow and its OpenGL 4.1 core, forward-compatible context. glfwInit()/glfwSetErrorCallback()/glfwTerminate() bracket every Window's lifetime but are the caller's responsibility (main.cpp) — a Window represents one window, not the GLFW library instance.
class Window {
public:
    Window(int width, int height, const std::string& title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    [[nodiscard]] bool shouldClose() const;
    void setShouldClose(bool shouldClose);
    void pollEvents() const;
    void swapBuffers() const;

    // For backends that need the raw GLFW handle (e.g. ImGui's GLFW backend) — everything else should use the typed accessors above.
    [[nodiscard]] GLFWwindow* nativeHandle() const noexcept { return window_; }

    // {width, height} in framebuffer pixels (glfwGetFramebufferSize), not screen points — the two differ by 2x on Retina displays. Queried fresh each call, not cached from a resize event.
    [[nodiscard]] std::pair<int, int> framebufferSize() const;

    // {width, height} in screen points (glfwGetWindowSize), same units as cursorPosition(). Use to scale cursor into framebufferSize()/image pixel space; don't divide cursorPosition() by framebufferSize() directly.
    [[nodiscard]] std::pair<int, int> windowSize() const;

    // Invoked on GLFW's key event (GLFW_PRESS/GLFW_RELEASE/GLFW_REPEAT). Scancode/mods aren't forwarded — no consumer needs them. Single callback slot, shared by every edge-triggered hotkey; WASD/QE need continuous per-frame state instead, so they use isKeyDown() below.
    using KeyCallback = std::function<void(int key, int action)>;
    void setKeyCallback(KeyCallback callback);

    // Polled key state (glfwGetKey), for continuous movement (WASD/QE) where an edge-triggered callback would only fire once per press.
    [[nodiscard]] bool isKeyDown(int key) const;

    // Invoked on GLFW's mouse-button event. Single slot, like KeyCallback.
    using MouseButtonCallback = std::function<void(int button, int action)>;
    void setMouseButtonCallback(MouseButtonCallback callback);

    // Polled cursor position in screen coordinates (glfwGetCursorPos). Orbit only needs a once-per-frame delta between consecutive polls, so a callback (like resize/key) isn't needed here.
    [[nodiscard]] std::pair<double, double> cursorPosition() const;

    // Hides and locks the cursor to the window (GLFW_CURSOR_DISABLED) while true, e.g. during an LMB-drag orbit; restores the normal cursor when false.
    void setCursorLocked(bool locked);

private:
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

    GLFWwindow* window_ = nullptr;
    KeyCallback keyCallback_;
    MouseButtonCallback mouseButtonCallback_;
};

}  // namespace engine::platform
