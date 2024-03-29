#ifndef DEFINES_H
#define DEFINES_H
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <functional>
#include <iomanip>

#define PI glm::pi<float>()
#define PI_2 glm::pi<float>()/2.0f
#define PRINT_FIXED_FLOAT(x, prec) std::fixed <<std::setprecision(prec)<<(x)

#define APPEND_VEC2(vector, glm_vec2) vector.insert(vector.end(), {glm_vec2.x, glm_vec2.y})
#define APPEND_VEC3(vector, glm_vec3) vector.insert(vector.end(), {glm_vec3.x, glm_vec3.y, glm_vec3.z})

const unsigned int window_width_g = 1280;
const unsigned int window_height_g = 960;
const bool window_full_screen_g = false;

typedef std::unordered_map<int, bool> KeyMap;
typedef std::unordered_map<int, bool> MouseMap;

struct Mouse {
    bool first_captured = true;
    bool captured = true;
    glm::vec2 prev;
    glm::vec2 move;
};

#define HEXCOLOR(h) {((h&0xFF0000)>>16)/255.0f, ((h&0x00FF00)>>8)/255.0f, (h&0x0000FF)/255.0f, 1.0f}
#define HEXCOLORALPH(h, a) {((h&0xFF0000)>>16)/255.0f, ((h&0x00FF00)>>8)/255.0f, (h&0x0000FF)/255.0f, a}
namespace Colors {
    const glm::vec4 White      = HEXCOLOR(0xFFFFFF);
    const glm::vec4 Black      = HEXCOLOR(0x000000);
    const glm::vec4 SlimeGreen = HEXCOLOR(0xAFAF00);
    const glm::vec4 Magenta = HEXCOLOR(0xFF00FF);
    const glm::vec4 Transparent = HEXCOLORALPH(0x000000, 0.0f);
    const glm::vec4 Yellow      = HEXCOLOR(0xFFFF00);
    const glm::vec4 SeaBlue     = HEXCOLOR(0x83A598);
}

typedef std::function<void(Mouse& mouse)> MouseHandler;
typedef std::function<void(double xoffset, double yoffset)> ScrollHandler;


template <typename K, typename V>
void overwrite_emplace(std::unordered_map<K, V>& map, K key, V&& val) {
    auto it = map.find(key);
    if(it != map.end()) {
        map.erase(key);
    }
    map.emplace(key, val);
}

#endif