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
    // Deliberately NOT setting GLFW_SRGB_CAPABLE: display encoding must
    // happen only in the OCIO shader (Stage F), never via a driver-level
    // sRGB framebuffer conversion. Do not add this hint in a later stage.

    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (window_ == nullptr) {
        std::cerr << "Window: glfwCreateWindow failed\n";
        std::exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window_);
    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &Window::framebufferSizeCallback);
    glfwSetKeyCallback(window_, &Window::keyCallback);
}

Window::~Window() {
    if (window_ != nullptr) {
        glfwDestroyWindow(window_);
    }
}

Window::Window(Window&& other) noexcept
    : window_(std::exchange(other.window_, nullptr)),
      resizeCallback_(std::move(other.resizeCallback_)),
      keyCallback_(std::move(other.keyCallback_)) {
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
        resizeCallback_ = std::move(other.resizeCallback_);
        keyCallback_ = std::move(other.keyCallback_);
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

void Window::setResizeCallback(ResizeCallback callback) {
    resizeCallback_ = std::move(callback);
}

void Window::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self != nullptr && self->resizeCallback_) {
        self->resizeCallback_(width, height);
    }
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

}  // namespace engine::platform
