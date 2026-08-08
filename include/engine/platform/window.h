#pragma once

#include <functional>
#include <string>
#include <utility>

// Forward-declared rather than #include <GLFW/glfw3.h>: a pointer to an
// incomplete type is sufficient here, and keeping GLFW out of this public
// header means consumers that only need shouldClose()/pollEvents()/etc.
// don't drag GLFW/GL declarations in with them.
struct GLFWwindow;

namespace engine::platform {

// Owns a single GLFWwindow and its OpenGL 4.1 core, forward-compatible
// context. glfwInit()/glfwSetErrorCallback()/glfwTerminate() bracket every
// Window's lifetime but are the caller's responsibility (main.cpp) — a
// Window represents one window, not the GLFW library instance.
class Window {
public:
    Window(int width, int height, const std::string& title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&& other) noexcept;
    Window& operator=(Window&& other) noexcept;

    [[nodiscard]] bool shouldClose() const;
    void pollEvents() const;
    void swapBuffers() const;

    // {width, height} in framebuffer pixels (glfwGetFramebufferSize), not
    // screen points — the two differ by 2x on Retina displays.
    [[nodiscard]] std::pair<int, int> framebufferSize() const;

    // Invoked on GLFW's framebuffer-resize event. Not consumed by Stage B;
    // Stage D's HDR framebuffer resize will use it.
    using ResizeCallback = std::function<void(int, int)>;
    void setResizeCallback(ResizeCallback callback);

private:
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

    GLFWwindow* window_ = nullptr;
    ResizeCallback resizeCallback_;
};

}  // namespace engine::platform
