#ifndef JSON_SERIALIZABLE_ABSTRACT_H
#define JSON_SERIALIZABLE_ABSTRACT_H
#include <string>
#include <memory>
#include "third_party/nlohmann_json/single_include/nlohmann/json.hpp"
using json = nlohmann::json;

class JSONSerializableAbstract{

public:
    virtual json getJSONObject() = 0;
};


#endif
