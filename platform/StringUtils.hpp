/**
 * @file platform/StringUtils.hpp
 * @brief Header-only string_view helpers for embedded command parsing.
 */

#ifndef PLATFORM_STRING_UTILS_HPP
#define PLATFORM_STRING_UTILS_HPP

#include <cstddef>
#include <string_view>

namespace platform::str {

/**
 * @brief Check if a character is a space or tab.
 * @param c The character to check.
 * @return true if the character is a space or tab, false otherwise.
 */
constexpr bool isSpaceOrTab(char c) noexcept { return c == ' ' || c == '\t'; }

/**
 * @brief Trim leading and trailing spaces and tabs from a string view.
 * @param s The string view to trim.
 * @return The trimmed string view.
 */
constexpr std::string_view trim(std::string_view s) noexcept {
  while (!s.empty() && isSpaceOrTab(s.front())) {
    s.remove_prefix(1);
  }
  while (!s.empty() && isSpaceOrTab(s.back())) {
    s.remove_suffix(1);
  }
  return s;
}

/**
 * @brief Get the next token from a string view.
 * @param rest The string view to get the next token from.
 * @return The next token.
 */
constexpr std::string_view nextToken(std::string_view &rest) noexcept {
  rest = trim(rest);
  if (rest.empty()) {
    return {};
  }

  std::size_t tokenLen = 0u;
  while (tokenLen < rest.size() && !isSpaceOrTab(rest[tokenLen])) {
    ++tokenLen;
  }

  const std::string_view token = rest.substr(0u, tokenLen);
  rest.remove_prefix(tokenLen);
  return token;
}

/**
 * @brief Convert a lowercase ASCII character to uppercase.
 * @param c The character to convert.
 * @return The uppercase character.
 */
constexpr char toUpperAscii(char c) noexcept {
  return (c >= 'a' && c <= 'z') ? static_cast<char>(c - ('a' - 'A')) : c;
}

/**
 * @brief Check if two string views are equal, ignoring case.
 * @param a The first string view.
 * @param b The second string view.
 * @return true if the string views are equal, ignoring case, false otherwise.
 */
constexpr bool iequals(std::string_view a, std::string_view b) noexcept {
  if (a.size() != b.size()) {
    return false;
  }

  for (std::size_t i = 0; i < a.size(); ++i) {
    if (toUpperAscii(a[i]) != toUpperAscii(b[i])) {
      return false;
    }
  }
  return true;
}

} // namespace platform::str

#endif // PLATFORM_STRING_UTILS_HPP
