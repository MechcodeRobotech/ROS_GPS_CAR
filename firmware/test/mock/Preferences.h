/**
 * Just enough Preferences for the host tests to compile.
 *
 * The registry takes a Preferences* and accepts nullptr, which is what the
 * tests pass - persistence is an ESP32 concern and there is no NVS here. This
 * exists so the header compiles, not so it behaves: a test that needs real
 * storage semantics would be testing NVS rather than our own logic.
 */
#ifndef MOCK_PREFERENCES_H
#define MOCK_PREFERENCES_H

#include <map>
#include <string>

class Preferences {
 public:
  bool begin(const char *, bool = false) { return true; }
  void end() {}
  bool isKey(const char *key) { return store_.count(key) > 0; }
  float getFloat(const char *key, float fallback = 0.0f) {
    auto it = store_.find(key);
    return it == store_.end() ? fallback : it->second;
  }
  size_t putFloat(const char *key, float value) {
    store_[key] = value;
    return sizeof(float);
  }
  bool remove(const char *key) { return store_.erase(key) > 0; }
  bool clear() { store_.clear(); return true; }

 private:
  std::map<std::string, float> store_;
};

#endif  // MOCK_PREFERENCES_H
