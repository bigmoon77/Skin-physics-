#include "skin_physics/serialization/json.hpp"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>
namespace skin_physics::serialization {
bool json_value::is_null() const{return std::holds_alternative<std::nullptr_t>(data);}
bool json_value::is_bool() const{return std::holds_alternative<bool>(data);}
bool json_value::is_number() const{return std::holds_alternative<double>(data);}
bool json_value::is_string() const{return std::holds_alternative<std::string>(data);}
bool json_value::is_array() const{return std::holds_alternative<array>(data);}
bool json_value::is_object() const{return std::holds_alternative<object>(data);}
const bool& json_value::as_bool() const{return std::get<bool>(data);}
const double& json_value::as_number() const{return std::get<double>(data);}
const std::string& json_value::as_string() const{return std::get<std::string>(data);}
const json_value::array& json_value::as_array() const{return std::get<array>(data);}
json_value::array& json_value::as_array(){return std::get<array>(data);}
const json_value::object& json_value::as_object() const{return std::get<object>(data);}
json_value::object& json_value::as_object(){return std::get<object>(data);}
const json_value* json_value::find(const std::string& key) const{if(!is_object())return nullptr;const auto it=as_object().find(key);return it==as_object().end()?nullptr:&it->second;}
json_value* json_value::find(const std::string& key){if(!is_object())return nullptr;auto& values=as_object();const auto it=values.find(key);return it==values.end()?nullptr:&it->second;}
namespace {
class parser {
public:
 explicit parser(const std::string& source):_source(source){}
 json_parse_result parse(){
  json_parse_result result;
  try{result.value=parse_value(0);skip();if(_position!=_source.size())fail("Trailing characters after JSON value.");result.success=true;}
  catch(const std::runtime_error& error){result.error_offset=_position;result.error_message=error.what();}
  return result;
 }
private:
 const std::string& _source; std::size_t _position=0;
 [[noreturn]] void fail(const std::string& message) const{throw std::runtime_error(message);}
 void skip(){while(_position<_source.size()){const char c=_source[_position];if(c!=' '&&c!='\t'&&c!='\r'&&c!='\n')break;++_position;}}
 bool consume(char c){skip();if(_position<_source.size()&&_source[_position]==c){++_position;return true;}return false;}
 uint32_t hex4(){if(_position+4>_source.size())fail("Incomplete Unicode escape.");uint32_t value=0;for(uint32_t i=0;i<4;++i){char c=_source[_position++];value<<=4;if(c>='0'&&c<='9')value|=uint32_t(c-'0');else if(c>='a'&&c<='f')value|=uint32_t(c-'a'+10);else if(c>='A'&&c<='F')value|=uint32_t(c-'A'+10);else fail("Invalid Unicode escape.");}return value;}
 static void utf8(std::string& out,uint32_t cp){if(cp<=0x7f)out.push_back(char(cp));else if(cp<=0x7ff){out.push_back(char(0xc0|(cp>>6)));out.push_back(char(0x80|(cp&63)));}else if(cp<=0xffff){out.push_back(char(0xe0|(cp>>12)));out.push_back(char(0x80|((cp>>6)&63)));out.push_back(char(0x80|(cp&63)));}else{out.push_back(char(0xf0|(cp>>18)));out.push_back(char(0x80|((cp>>12)&63)));out.push_back(char(0x80|((cp>>6)&63)));out.push_back(char(0x80|(cp&63)));}}
 std::string string_value(){
  if(!consume('"'))fail("Expected JSON string.");std::string out;
  while(_position<_source.size()){unsigned char c=static_cast<unsigned char>(_source[_position++]);if(c=='"')return out;if(c<0x20)fail("Unescaped control character.");if(c!='\\'){out.push_back(char(c));continue;}if(_position>=_source.size())fail("Incomplete string escape.");
   char e=_source[_position++];switch(e){case '"':out.push_back('"');break;case '\\':out.push_back('\\');break;case '/':out.push_back('/');break;case 'b':out.push_back('\b');break;case 'f':out.push_back('\f');break;case 'n':out.push_back('\n');break;case 'r':out.push_back('\r');break;case 't':out.push_back('\t');break;case 'u':{uint32_t cp=hex4();if(cp>=0xd800&&cp<=0xdbff){if(_position+2>_source.size()||_source[_position]!='\\'||_source[_position+1]!='u')fail("High surrogate missing low surrogate.");_position+=2;uint32_t low=hex4();if(low<0xdc00||low>0xdfff)fail("Invalid low surrogate.");cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00);}else if(cp>=0xdc00&&cp<=0xdfff)fail("Unexpected low surrogate.");utf8(out,cp);break;}default:fail("Unknown string escape.");}
  }fail("Unterminated JSON string.");
 }
 json_value number_value(){
  const std::size_t start=_position;if(_source[_position]=='-')++_position;if(_position>=_source.size())fail("Incomplete number.");
  if(_source[_position]=='0'){++_position;if(_position<_source.size()&&_source[_position]>='0'&&_source[_position]<='9')fail("Leading zero in number.");}
  else{if(_source[_position]<'1'||_source[_position]>'9')fail("Invalid integer part.");while(_position<_source.size()&&_source[_position]>='0'&&_source[_position]<='9')++_position;}
  if(_position<_source.size()&&_source[_position]=='.'){++_position;std::size_t begin=_position;while(_position<_source.size()&&_source[_position]>='0'&&_source[_position]<='9')++_position;if(begin==_position)fail("Fraction requires digits.");}
  if(_position<_source.size()&&(_source[_position]=='e'||_source[_position]=='E')){++_position;if(_position<_source.size()&&(_source[_position]=='+'||_source[_position]=='-'))++_position;std::size_t begin=_position;while(_position<_source.size()&&_source[_position]>='0'&&_source[_position]<='9')++_position;if(begin==_position)fail("Exponent requires digits.");}
  std::string text=_source.substr(start,_position-start);char* end=nullptr;double number=std::strtod(text.c_str(),&end);if(end==text.c_str()||*end!='\0'||!std::isfinite(number))fail("Number is not finite or representable.");json_value result;result.data=number;return result;
 }
 json_value array_value(uint32_t depth){consume('[');json_value::array out;if(consume(']')){json_value v;v.data=std::move(out);return v;}while(true){out.push_back(parse_value(depth+1));if(consume(']'))break;if(!consume(','))fail("Expected comma in array.");}json_value v;v.data=std::move(out);return v;}
 json_value object_value(uint32_t depth){consume('{');json_value::object out;if(consume('}')){json_value v;v.data=std::move(out);return v;}while(true){std::string key=string_value();if(!consume(':'))fail("Expected colon after object key.");json_value value=parse_value(depth+1);if(!out.emplace(key,std::move(value)).second)fail("Duplicate JSON object key.");if(consume('}'))break;if(!consume(','))fail("Expected comma in object.");}json_value v;v.data=std::move(out);return v;}
 json_value parse_value(uint32_t depth){if(depth>256)fail("JSON nesting exceeds 256.");skip();if(_position>=_source.size())fail("Unexpected end of JSON.");char c=_source[_position];if(c=='{')return object_value(depth);if(c=='[')return array_value(depth);if(c=='"'){json_value v;v.data=string_value();return v;}if(c=='-'||(c>='0'&&c<='9'))return number_value();if(_source.compare(_position,4,"true")==0){_position+=4;json_value v;v.data=true;return v;}if(_source.compare(_position,5,"false")==0){_position+=5;json_value v;v.data=false;return v;}if(_source.compare(_position,4,"null")==0){_position+=4;return {};}fail("Unexpected JSON token.");}
};
void indent(std::string& out,uint32_t depth,uint32_t width){out.append(std::size_t(depth)*width,' ');}
void quote(std::string& out,const std::string& value){static const char hex[]="0123456789abcdef";out.push_back('"');for(unsigned char c:value){switch(c){case '"':out+="\\\"";break;case '\\':out+="\\\\";break;case '\b':out+="\\b";break;case '\f':out+="\\f";break;case '\n':out+="\\n";break;case '\r':out+="\\r";break;case '\t':out+="\\t";break;default:if(c<32){out+="\\u00";out.push_back(hex[c>>4]);out.push_back(hex[c&15]);}else out.push_back(char(c));}}out.push_back('"');}
bool write_value(std::string& out,const json_value& value,uint32_t depth,uint32_t width,std::string& error){
 if(value.is_null()){out+="null";return true;}if(value.is_bool()){out+=value.as_bool()?"true":"false";return true;}
 if(value.is_number()){if(!std::isfinite(value.as_number())){error="Cannot serialize non-finite number.";return false;}std::ostringstream s;s.imbue(std::locale::classic());s<<std::setprecision(17)<<value.as_number();out+=s.str();return true;}
 if(value.is_string()){quote(out,value.as_string());return true;}
 if(value.is_array()){out.push_back('[');const auto& a=value.as_array();if(!a.empty()){out.push_back('\n');for(std::size_t i=0;i<a.size();++i){indent(out,depth+1,width);if(!write_value(out,a[i],depth+1,width,error))return false;if(i+1<a.size())out.push_back(',');out.push_back('\n');}indent(out,depth,width);}out.push_back(']');return true;}
 if(value.is_object()){out.push_back('{');const auto& o=value.as_object();if(!o.empty()){out.push_back('\n');std::size_t i=0;for(const auto& entry:o){indent(out,depth+1,width);quote(out,entry.first);out+=": ";if(!write_value(out,entry.second,depth+1,width,error))return false;if(++i<o.size())out.push_back(',');out.push_back('\n');}indent(out,depth,width);}out.push_back('}');return true;}error="Unknown JSON value.";return false;
}
}
json_parse_result parse_json(const std::string& source){return parser(source).parse();}
json_write_result write_json(const json_value& value,uint32_t indentation){json_write_result r;if(indentation>16){r.error_message="Indentation exceeds 16.";return r;}if(write_value(r.text,value,0,indentation,r.error_message)){r.text.push_back('\n');r.success=true;}else r.text.clear();return r;}
bool read_text_file(const std::filesystem::path& path,std::string& output,std::string& error){std::ifstream file(path,std::ios::binary);if(!file){error="Could not open file: "+path.string();return false;}std::istreambuf_iterator<char> begin(file),end;output.assign(begin,end);if(file.bad()){error="Failed reading file: "+path.string();output.clear();return false;}return true;}
bool write_text_file(const std::filesystem::path& path,const std::string& contents,std::string& error){std::ofstream file(path,std::ios::binary|std::ios::trunc);if(!file){error="Could not open file: "+path.string();return false;}file.write(contents.data(),static_cast<std::streamsize>(contents.size()));if(!file.good()){error="Failed writing file: "+path.string();return false;}return true;}
}