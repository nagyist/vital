/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KWIVER_VITAL_PLUGIN_MANAGER_H_
#define KWIVER_VITAL_PLUGIN_MANAGER_H_

#include <vital/vital_types.h>

#include <vector>
#include <string>
#include <map>
#include <memory>


namespace kwiver {
namespace vital {

// base class of factory hierarchy
class plugin_factory;

typedef std::shared_ptr< plugin_factory >         plugin_factory_handle_t;
typedef std::vector< plugin_factory_handle_t >    plugin_factory_vector_t;
typedef std::map< std::string, plugin_factory_vector_t > plugin_map_t;

class plugin_manager_impl;

// ----------------------------------------------------------------
/**
 * @brief Manages a set of plugin (singleton)
 *
 * The plugin manager keeps track of all factories from plugins that
 * are discovered on the disk. The list of directories that are
 * searched is specified by the VITAL_MODULE_PATH environment variable.
 *
 *
 */
class plugin_manager
{
public:
  static plugin_manager* instance();
  plugin_manager();
  ~plugin_manager();

  /**
   * @brief Add an additional directory to search for plugins in.
   *
   * This method adds the specified directory to the list used when
   * loading plugins. This method can be called multiple times to add
   * multiple directories. Call the register_plugins() method to load
   * plugins after you have added all additional directories.
   *
   * Directory paths that don't exist will simply be ignored.
   *
   * \param dirpath Path to the directory to add to the plugin search path
   */
  void add_search_path(path_t const& dirpath);

  /**
   * @brief Get plugin manager search path
   *
   *  This method returns the search path used to load algorithms.
   *
   * @return vector of paths that are searched
   */
  std::vector< path_t > const& get_search_path() const;

  /**
   * @brief Get list of factories for interface type.
   *
   * This method returns a list of pointer to factory methods that
   * create objects of the desired interface type.
   *
   * @param type_name Type name of the interface required
   *
   * @return Vector of factories. (vector may be empty)
   */
  plugin_factory_vector_t const& get_factories( std::string const& type_name ) const;

  /**
   * @brief Add factory to manager.
   *
   * This method adds the specified plugin factory to the plugin
   * manager. This method is usually called from the plugin
   * registration function in the loadable module to self-register all
   * plugins in a module.
   *
   * Plugin factory objects are grouped under the interface type name,
   * so all factories that create the same interface are together.
   *
   * @param fact Plugin factory object to register
   *
   * @return A pointer is returned to the added factory in case
   * attributes need to be added to the factory.
   *
   * Example:
   \code
   void add_factories( plugin_manager* pm )
   {
     //                                              interface type    concrete type
     plugin_factory_handle_t fact = pm->ADD_FACTORY( reader_interface, reader_nit );
     fact.add_attribute( "file-type", "xml mit" );
   }
   \endcode
   */
  plugin_factory_handle_t add_factory( plugin_factory* fact );

  /**
   * @brief Get map of known plugins.
   *
   * Get the map of all known registered plugins.
   *
   * @return Map of plugins
   */
  plugin_map_t const& get_plugin_map() const;

  /**
   * @brief Get list of files loaded.
   *
   * This method returns the list of shared object file names that
   * successfully re loaded.
   *
   * @return List of file names.
   */
  std::vector< std::string > get_file_list() const;

private:
  const std::unique_ptr< plugin_manager_impl > m_impl;
  static plugin_manager* s_instance;

}; // end class plugin_manager

} } // end namespace

#endif /* KWIVER_VITAL_PLUGIN_MANAGER_H_ */
