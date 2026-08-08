#version 410 core

in vec2 vUv;
out vec4 fragColor;

uniform sampler2D uAlbedo;

void main() {
    fragColor = texture(uAlbedo, vUv);
}
