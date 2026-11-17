#pragma once

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace test_json
{
    struct value
    {
        using array = std::vector<value>;
        using object = std::map<std::string, value>;
        std::variant<std::nullptr_t, bool, double, std::string, array, object> data = nullptr;

        const value& at(const std::string& key) const
        {
            const object& values = std::get<object>(data);
            const auto iterator = values.find(key);
            if (iterator == values.end())
            {
                throw std::runtime_error("Missing JSON field: " + key);
            }
            return iterator->second;
        }

        const value& at(std::size_t index) const
        {
            const array& values = std::get<array>(data);
            if (index >= values.size())
            {
                throw std::runtime_error("JSON array index out of range.");
            }
            return values[index];
        }

        const array& as_array() const { return std::get<array>(data); }
        const object& as_object() const { return std::get<object>(data); }
        const std::string& as_string() const { return std::get<std::string>(data); }
        double as_number() const { return std::get<double>(data); }
        bool as_bool() const { return std::get<bool>(data); }
    };

    class parser
    {
    public:
        explicit parser(const std::string& source) : _source(source) {}

        value parse()
        {
            value result = parse_value();
            skip_whitespace();
            if (_position != _source.size())
            {
                fail("Trailing characters after JSON value.");
            }
            return result;
        }

    private:
        const std::string& _source;
        std::size_t _position = 0;

        [[noreturn]] void fail(const std::string& message) const
        {
            throw std::runtime_error("JSON parse error at byte " +
                std::to_string(_position) + ": " + message);
        }

        void skip_whitespace()
        {
            while (_position < _source.size() &&
                std::isspace(static_cast<unsigned char>(_source[_position])) != 0)
            {
                ++_position;
            }
        }

        bool consume(char expected)
        {
            skip_whitespace();
            if (_position < _source.size() && _source[_position] == expected)
            {
                ++_position;
                return true;
            }
            return false;
        }

        std::string parse_string()
        {
            if (!consume('"')) fail("Expected string.");
            std::string output;
            while (_position < _source.size())
            {
                const char current = _source[_position++];
                if (current == '"') return output;
                if (current != '\\')
                {
                    output.push_back(current);
                    continue;
                }
                if (_position >= _source.size()) fail("Incomplete string escape.");
                const char escaped = _source[_position++];
                switch (escaped)
                {
                case '"': output.push_back('"'); break;
                case '\\': output.push_back('\\'); break;
                case '/': output.push_back('/'); break;
                case 'b': output.push_back('\b'); break;
                case 'f': output.push_back('\f'); break;
                case 'n': output.push_back('\n'); break;
                case 'r': output.push_back('\r'); break;
                case 't': output.push_back('\t'); break;
                case 'u':
                    if (_position + 4 > _source.size()) fail("Incomplete unicode escape.");
                    // The project fixtures use ASCII JSON strings; preserve non-ASCII code points as '?'.
                    output.push_back('?');
                    _position += 4;
                    break;
                default: fail("Unknown string escape.");
                }
            }
            fail("Unterminated string.");
        }

        value parse_number()
        {
            skip_whitespace();
            const char* begin = _source.c_str() + _position;
            char* end = nullptr;
            const double number = std::strtod(begin, &end);
            if (end == begin || !std::isfinite(number)) fail("Invalid or non-finite number.");
            _position += static_cast<std::size_t>(end - begin);
            value result;
            result.data = number;
            return result;
        }

        value parse_array()
        {
            consume('[');
            value::array output;
            if (consume(']'))
            {
                value result;
                result.data = std::move(output);
                return result;
            }
            do
            {
                output.push_back(parse_value());
            } while (consume(','));
            if (!consume(']')) fail("Expected closing array bracket.");
            value result;
            result.data = std::move(output);
            return result;
        }

        value parse_object()
        {
            consume('{');
            value::object output;
            if (consume('}'))
            {
                value result;
                result.data = std::move(output);
                return result;
            }
            do
            {
                const std::string key = parse_string();
                if (!consume(':')) fail("Expected colon after object key.");
                output[key] = parse_value();
            } while (consume(','));
            if (!consume('}')) fail("Expected closing object brace.");
            value result;
            result.data = std::move(output);
            return result;
        }

        value parse_value()
        {
            skip_whitespace();
            if (_position >= _source.size()) fail("Unexpected end of input.");
            const char current = _source[_position];
            if (current == '{') return parse_object();
            if (current == '[') return parse_array();
            if (current == '"')
            {
                value result;
                result.data = parse_string();
                return result;
            }
            if (current == '-' || (current >= '0' && current <= '9')) return parse_number();
            if (_source.compare(_position, 4, "true") == 0)
            {
                _position += 4;
                value result;
                result.data = true;
                return result;
            }
            if (_source.compare(_position, 5, "false") == 0)
            {
                _position += 5;
                value result;
                result.data = false;
                return result;
            }
            if (_source.compare(_position, 4, "null") == 0)
            {
                _position += 4;
                return {};
            }
            fail("Unexpected token.");
        }
    };

    inline value load(const std::filesystem::path& path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            throw std::runtime_error("Could not open JSON fixture: " + path.string());
        }
        const std::string source(
            std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>());
        return parser(source).parse();
    }

    inline double number(const value& value)
    {
        return value.as_number();
    }

    inline bool nearly_equal(double left, double right, double tolerance)
    {
        return std::abs(left - right) <= tolerance;
    }
}
