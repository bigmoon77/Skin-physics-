#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <variant>
#include <vector>
namespace skin_physics::serialization {
struct json_value {
 using array=std::vector<json_value>;
 using object=std::map<std::string,json_value>;
 using storage=std::variant<std::nullptr_t,bool,double,std::string,array,object>;
 storage data=nullptr;
 bool is_null() const; bool is_bool() const; bool is_number() const; bool is_string() const; bool is_array() const; bool is_object() const;
 const bool& as_bool() const; const double& as_number() const; const std::string& as_string() const;
 const array& as_array() const; array& as_array(); const object& as_object() const; object& as_object();
 const json_value* find(const std::string& key) const; json_value* find(const std::string& key);
 friend bool operator==(const json_value& a,const json_value& b){return a.data==b.data;}
};
struct json_parse_result { json_value value; std::size_t error_offset=0; std::string error_message; bool success=false; };
struct json_write_result { std::string text; std::string error_message; bool success=false; };
json_parse_result parse_json(const std::string& source);
json_write_result write_json(const json_value& value,uint32_t indentation=2);
bool read_text_file(const std::filesystem::path& path,std::string& output,std::string& error_message);
bool write_text_file(const std::filesystem::path& path,const std::string& contents,std::string& error_message);
}