#include "engine/platform/window.h"

#include <cstdlib>
#include <iostream>

#include <GLFW/glfw3.h>

namespace engine::platform {

Window::Window(int width, int height, const std::string& title) {
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);  // mandatory on macOS
    // Deliberately NOT setting GLFW_SRGB_CAPABLE: display encoding must happen only in the OCIO shader (Stage F), never via a driver-level sRGB framebuffer conversion. Do not add this hint in a later stage.

    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (window_ == nullptr) {
        std::cerr << "Window: glfwCreateWindow failed\n";
        std::exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, &Window::keyCallback);
    glfwSetMouseButtonCallback(window_, &Window::mouseButtonCallback);
}

Window::~Window() {
    if (window_ != nullptr) {
        glfwDestroyWindow(window_);
    }
}

Window::Window(Window&& other) noexcept
    : window_(std::exchange(other.window_, nullptr)),
      keyCallback_(std::move(other.keyCallback_)),
      mouseButtonCallback_(std::move(other.mouseButtonCallback_)) {
    if (window_ != nullptr) {
        glfwSetWindowUserPointer(window_, this);
    }
}

Window& Window::operator=(Window&& other) noexcept {
    if (this != &other) {
        if (window_ != nullptr) {
            glfwDestroyWindow(window_);
        }
        window_ = std::exchange(other.window_, nullptr);
        keyCallback_ = std::move(other.keyCallback_);
        mouseButtonCallback_ = std::move(other.mouseButtonCallback_);
        if (window_ != nullptr) {
            glfwSetWindowUserPointer(window_, this);
        }
    }
    return *this;
}

bool Window::shouldClose() const {
    return glfwWindowShouldClose(window_) != 0;
}

void Window::pollEvents() const {
    glfwPollEvents();
}

void Window::swapBuffers() const {
    glfwSwapBuffers(window_);
}

std::pair<int, int> Window::framebufferSize() const {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    return {width, height};
}

void Window::setKeyCallback(KeyCallback callback) {
    keyCallback_ = std::move(callback);
}

void Window::keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action,
                          int /*mods*/) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self != nullptr && self->keyCallback_) {
        self->keyCallback_(key, action);
    }
}

bool Window::isKeyDown(int key) const {
    return glfwGetKey(window_, key) == GLFW_PRESS;
}

void Window::setMouseButtonCallback(MouseButtonCallback callback) {
    mouseButtonCallback_ = std::move(callback);
}

std::pair<double, double> Window::cursorPosition() const {
    double x = 0.0;
    double y = 0.0;
    glfwGetCursorPos(window_, &x, &y);
    return {x, y};
}

void Window::setCursorLocked(bool locked) {
    glfwSetInputMode(window_, GLFW_CURSOR, locked ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

void Window::mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self != nullptr && self->mouseButtonCallback_) {
        self->mouseButtonCallback_(button, action);
    }
}

}  // namespace engine::platform
