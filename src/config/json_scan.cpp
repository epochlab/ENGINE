#include "json_scan.h"

#include <charconv>
#include <fstream>
#include <sstream>

namespace engine::config::json {

namespace {

bool isJsonWhitespace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// Finds "key" as a quoted JSON key anywhere in text, returning the
// position right after the closing quote, or npos.
std::size_t findKeyEnd(std::string_view text, std::string_view key) {
    const std::string needle = "\"" + std::string(key) + "\"";
    const std::size_t pos = text.find(needle);
    if (pos == std::string_view::npos) {
        return std::string_view::npos;
    }
    return pos + needle.size();
}

// Given the position right after a key's closing quote, requires the
// next non-whitespace character to be ':' (rejecting malformed input
// rather than scanning ahead into a neighboring field's colon) and skips
// whitespace after it too, returning the position of the value's first
// character, or npos if a ':' isn't there.
std::size_t findValueStart(std::string_view text, std::size_t keyEnd) {
    std::size_t pos = keyEnd;
    while (pos < text.size() && isJsonWhitespace(text[pos])) {
        ++pos;
    }
    if (pos >= text.size() || text[pos] != ':') {
        return std::string_view::npos;
    }
    ++pos;
    while (pos < text.size() && isJsonWhitespace(text[pos])) {
        ++pos;
    }
    return pos;
}

}  // namespace

std::optional<std::string> readFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return std::nullopt;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::optional<double> findNumber(std::string_view text, std::string_view key) {
    const std::size_t keyEnd = findKeyEnd(text, key);
    if (keyEnd == std::string_view::npos) {
        return std::nullopt;
    }
    const std::size_t valueStart = findValueStart(text, keyEnd);
    if (valueStart == std::string_view::npos) {
        return std::nullopt;
    }
    double value = 0.0;
    const std::from_chars_result result =
        std::from_chars(text.data() + valueStart, text.data() + text.size(), value);
    if (result.ec != std::errc{}) {
        return std::nullopt;
    }
    return value;
}

std::optional<std::string> findString(std::string_view text, std::string_view key) {
    const std::size_t keyEnd = findKeyEnd(text, key);
    if (keyEnd == std::string_view::npos) {
        return std::nullopt;
    }
    const std::size_t valueStart = findValueStart(text, keyEnd);
    if (valueStart == std::string_view::npos || valueStart >= text.size() ||
        text[valueStart] != '"') {
        return std::nullopt;
    }
    const std::size_t contentStart = valueStart + 1;
    const std::size_t valueEnd = text.find('"', contentStart);
    if (valueEnd == std::string_view::npos) {
        return std::nullopt;
    }
    return std::string(text.substr(contentStart, valueEnd - contentStart));
}

std::optional<glm::vec3> findVec3(std::string_view text, std::string_view key) {
    const std::size_t keyEnd = findKeyEnd(text, key);
    if (keyEnd == std::string_view::npos) {
        return std::nullopt;
    }
    const std::size_t valueStart = findValueStart(text, keyEnd);
    if (valueStart == std::string_view::npos || valueStart >= text.size() ||
        text[valueStart] != '[') {
        return std::nullopt;
    }
    const std::size_t valueEnd = text.find(']', valueStart);
    if (valueEnd == std::string_view::npos) {
        return std::nullopt;
    }
    const std::string_view inner = text.substr(valueStart + 1, valueEnd - valueStart - 1);

    glm::vec3 result{};
    std::size_t pos = 0;
    for (int component = 0; component < 3; ++component) {
        while (pos < inner.size() && (isJsonWhitespace(inner[pos]) || inner[pos] == ',')) {
            ++pos;
        }
        if (pos >= inner.size()) {
            return std::nullopt;
        }
        float value = 0.0F;
        const std::from_chars_result parseResult =
            std::from_chars(inner.data() + pos, inner.data() + inner.size(), value);
        if (parseResult.ec != std::errc{}) {
            return std::nullopt;
        }
        result[component] = value;
        pos = static_cast<std::size_t>(parseResult.ptr - inner.data());
    }
    return result;
}

std::string_view findObjectBody(std::string_view text, std::string_view key) {
    const std::size_t keyEnd = findKeyEnd(text, key);
    if (keyEnd == std::string_view::npos) {
        return {};
    }
    const std::size_t valueStart = findValueStart(text, keyEnd);
    if (valueStart == std::string_view::npos || valueStart >= text.size() ||
        text[valueStart] != '{') {
        return {};
    }
    // Tracks nesting depth while scanning for the matching closing brace.
    // filmBack/light are one level deep with no further nested objects
    // today, but depth-tracking costs nothing and avoids a subtle bug if
    // that ever changes.
    int depth = 0;
    for (std::size_t pos = valueStart; pos < text.size(); ++pos) {
        if (text[pos] == '{') {
            ++depth;
        } else if (text[pos] == '}') {
            --depth;
            if (depth == 0) {
                return text.substr(valueStart + 1, pos - valueStart - 1);
            }
        }
    }
    return {};
}

std::vector<std::string_view> findObjectArrayBodies(std::string_view text, std::string_view key) {
    const std::size_t keyEnd = findKeyEnd(text, key);
    if (keyEnd == std::string_view::npos) {
        return {};
    }
    const std::size_t valueStart = findValueStart(text, keyEnd);
    if (valueStart == std::string_view::npos || valueStart >= text.size() ||
        text[valueStart] != '[') {
        return {};
    }

    std::vector<std::string_view> bodies;
    // Depth over braces only (not the enclosing brackets) -- an object's
    // start/end is tracked the same way findObjectBody tracks one, just
    // repeated for each element found before the array's closing ']'.
    int braceDepth = 0;
    std::size_t objectStart = 0;
    for (std::size_t pos = valueStart + 1; pos < text.size(); ++pos) {
        const char c = text[pos];
        if (c == ']' && braceDepth == 0) {
            return bodies;
        }
        if (c == '{') {
            if (braceDepth == 0) {
                objectStart = pos;
            }
            ++braceDepth;
        } else if (c == '}') {
            --braceDepth;
            if (braceDepth == 0) {
                bodies.push_back(text.substr(objectStart + 1, pos - objectStart - 1));
            }
        }
    }
    // Unterminated array (no closing ']' found) -- return whatever
    // complete objects were parsed rather than nothing, matching
    // findObjectBody's tolerance of malformed trailing input.
    return bodies;
}

}  // namespace engine::config::json
