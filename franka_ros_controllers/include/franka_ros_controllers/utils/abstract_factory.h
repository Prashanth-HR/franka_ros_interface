#pragma once

#include <franka_ros_controllers/utils/lockable_ptr.h>

#include <cxxabi.h>
#include <algorithm>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

namespace franka_ros_controllers {

template <class MapType>
std::vector<typename MapType::key_type> getMapKeys(const MapType& map) {
  std::vector<typename MapType::key_type> result;
  result.reserve(map.size());
  std::transform(std::begin(map), std::end(map), std::back_inserter(result),
                 [](auto& pair) { return pair.first; });
  return result;
}

/**
 * Type-trait that signals if a given class is streamable. Useful for template function that try to
 * stream a template parameter.
 */
template <typename S, typename T>
class is_streamable {
  template <typename SS, typename TT>
  static auto test(int) -> decltype(std::declval<SS&>() << std::declval<TT>(), std::true_type());

  template <typename, typename>
  static auto test(...) -> std::false_type;

 public:
  static const bool value = decltype(test<S, T>(0))::value;
};

/**
 * A template that can be used as a superclass of a class hierarchy that
 * wants to provide a factory method which allows instantiation of objects
 * based on a string identifier.
 *
 * The first template argument is the shared pointer type that will be created by the factory
 * functions. The second argument specifies the type of the key to be used for identifying the
 * objects (often std::string). Variadic template arguments can be used to specify arguments for the
 * construction of the factory objects.
 *
 * Usage:
 * @verbatim
 * // do typedef for your factory in some header
 * using MyFactory = AbstractFactory<MyBaseType>;
 *
 * // register your object globally (needs to be done in cpp)
 * static auto registration =
 *     MyFactory::registerClass<MyObjectType1>(
 *         "MyObjectType1");
 * static auto registration2 =
 *     MyFactory::registerClass<MyObjectType2>(
 *         "MyObjectType2");
 *
 * // request object creation anywhere in your code
 * MyFactory::SharedPointerType myObjectInstance = MyFactory::create("MyObjectType2");
 * @endverbatim
 */
template <typename ObjectBaseType, typename KeyType = std::string, typename... constructorArg>
class AbstractFactory {
 public:
  using SharedPointerType = std::shared_ptr<ObjectBaseType>;

  /**
   * The function pointer type of subclass initialisation functions.
   * This matches the createInstance method.
   */
  typedef SharedPointerType (*initialisationFunction)(constructorArg...);

  /**
   * Function which can be used to retrieve an object specified by a key.
   * @param[in] key Identifier for a specific object type
   * @param[in] params var-arg parameters passed on to the creation function of the object.
   */
  static SharedPointerType create(const KeyType& key,
                                  bool throwIfMissing = true,
                                  constructorArg... params) {
    auto types = subTypes();  // lock map
    auto it = types->find(key);
    if (it == types->end()) {
      if (throwIfMissing) {
        throw std::invalid_argument(createMissingFactoryErrorString(key).c_str());
      }
      return SharedPointerType();
    }

    SharedPointerType result = it->second(std::forward<constructorArg>(params)...);
    return result;
  }

  /**
   * Retrieves a list of all registered classes as their string-identifier aka name.
   * @return list of available classes by their name.
   */
  static std::vector<KeyType> getAvailableTypes() {
    auto types = subTypes();  // lock map
    return getMapKeys(*types);
  }

  /**
   * Initialisation function template which is usually used to store the function pointer for a type
   * in the registry. It calls the constructor and returns a shared_ptr to the resulting object.
   * @see registerClass()
   */
  template <typename ObjectType>
  static SharedPointerType createInstance(constructorArg... args) {
    return std::make_shared<ObjectType>(std::forward<constructorArg>(args)...);
  }

  /**
   * A helper struct to allow static initialisation of the subclass lookup table.
   */
  struct RegistryHelper {};

  /**
   * Statically called by subclasses to register their key and initialisation
   * function so they can be found by {@link create create}.
   */
  static RegistryHelper registerClass(const KeyType& key,
                                      initialisationFunction init,
                                      bool force = false) {
    std::cout << "Factory with base " << baseTypeString() << ": Registering type '"
              << keyToString(key) << "'" << std::endl;
    auto types = subTypes();
    if (force || types->count(key) == 0) {
      (*types)[key] = init;
    } else {
      throw std::invalid_argument(baseTypeString() + ": Key '" + keyToString(key) +
                                  "' exists already!");
    }
    return RegistryHelper();
  }

  /**
   * Registers a class under the given key id.
   *
   * @param[in] key The key under which to register the class type.
   * @return a registry helper.
   */
  template <typename ObjectType>
  static RegistryHelper registerClass(const KeyType& key, bool force = true) {
    registerClass(key, &createInstance<ObjectType>, force);
    return RegistryHelper();
  }

  using MapType = std::map<KeyType, initialisationFunction>;
  using MapTypePtr = typename lockable_ptr<MapType>::Ptr;

 private:
  static std::string baseTypeString() {
    int status = -1;
    char* demangledBaseName =
        abi::__cxa_demangle(typeid(ObjectBaseType).name(), nullptr, nullptr, &status);
    std::string result = (demangledBaseName ? demangledBaseName : typeid(ObjectBaseType).name());
    free(demangledBaseName);
    return result;
  }
  template <typename Key = KeyType,
            typename std::enable_if_t<!is_streamable<std::stringstream, Key>::value>* = nullptr>
  static std::string keyToString(const KeyType&) {
    return "<UnstreamableKey>";
  }

  template <typename Key = KeyType,
            typename std::enable_if_t<is_streamable<std::stringstream, Key>::value>* = nullptr>
  static std::string keyToString(const KeyType& key) {
    std::stringstream s;
    s << key;
    return s.str();
  }

  template <typename Key = KeyType,
            typename std::enable_if_t<!is_streamable<std::stringstream, Key>::value>* = nullptr>
  static std::string createMissingFactoryErrorString(const KeyType& key) {
    std::stringstream s;
    s << "Could not find object for factory " << typeid(ObjectBaseType*).name()
      << " with non-streamable key-type '" << typeid(key).name() << "'";
    return s.str();
  }

  template <typename Key = KeyType,
            typename std::enable_if_t<is_streamable<std::stringstream, Key>::value>* = nullptr>
  static std::string createMissingFactoryErrorString(const KeyType& key) {
    std::stringstream s;
    auto types = getAvailableTypes();
    s << baseTypeString() << ": Could not find object of type '" << key << "' - Available types ("
      << types.size() << ") :\n";
    if (!types.empty()) {
      for (auto& type : types) {
        s << "\t" << type << "\n";
      }
    } else {
      s << "<None>\n";
    }
    return s.str();
  }

  /**
   * Static wrapper method for accessing subTypes map.
   * This method is necessary to make certain that the map is initialised
   * before use. This can only be guaranteed through a static local variable
   * in a function. Returns a locked shared_ptr. The lock is active as long as the shared_ptr
   * instance is alive.
   */
  static MapTypePtr subTypes() {
    static lockable_ptr<MapType> subTypes;
    return subTypes.lock();
  }
};
}  // namespace franka_ros_controllers