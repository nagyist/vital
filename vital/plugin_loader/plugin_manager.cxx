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

#include "plugin_manager.h"
#include "plugin_factory.h"

#include <vital/algorithm_plugin_manager_paths.h>

#include <vital/logger/logger.h>

#include <sstream>
#include <mutex>


#include <kwiversys/SystemTools.hxx>
#include <kwiversys/DynamicLoader.hxx>
#include <kwiversys/Directory.hxx>

//+ kwiversys has a demangle but it is private
#if defined __linux__
  #include <cxxabi.h>
#endif


namespace kwiver {
namespace vital {

namespace {

typedef kwiversys::SystemTools     ST;
typedef kwiversys::DynamicLoader   DL;
typedef DL::LibraryHandle          library_t;
typedef DL::SymbolPointer          function_t;


static char const* environment_variable_name( "VITAL_PLUGIN_PATH" );

// Platform specific plugin library file (set as compile definition in CMake)
static std::string const shared_library_suffix = std::string( SHARED_LIB_SUFFIX );

// Default module directory locations. Values defined in CMake configuration.
static std::string const default_module_paths = std::string( DEFAULT_MODULE_PATHS );

} // end anon namespace



// ================================================================
//
// Static data for singleton
//
plugin_manager* plugin_manager::s_instance = 0;


// ==================================================================
/**
 * @brief Plugin manager private implementation.
 *
 */
class plugin_manager_impl
{
public:
  plugin_manager_impl( plugin_manager* parent)
    : m_parent( parent ),
      m_logger( kwiver::vital::get_logger( "vital.plugin_manager" ) )
  { }

  ~plugin_manager_impl() { }

  void load_known_modules();
  void look_in_directory( std::string const& directory);
  void load_from_module( std::string const& path);

  void print( std::ostream& str ) const;

  plugin_manager* m_parent;

  /// Paths in which to search for module libraries
  typedef std::vector< path_t > search_paths_t;
  search_paths_t m_search_paths;


  // Map from interface type name to vector of class loaders
  plugin_map_t m_plugin_map;

  // Map to keep track of the modules we have opened and loaded.
  typedef std::map< std::string, kwiversys::DynamicLoader::LibraryHandle > library_map_t;
  library_map_t m_library_map;

  // Name of current module file we are processing
  std::string m_current_filename;

  kwiver::vital::logger_handle_t m_logger;

}; // end class plugin_manager_impl


// ==================================================================
plugin_manager*
plugin_manager::
instance()
{
  static std::mutex instance_lock;

  if ( 0 == s_instance )
  {
    std::lock_guard< std::mutex > lock( instance_lock );

    if ( 0 == s_instance )
    {
      s_instance = new plugin_manager();
    }
  }

  return s_instance;
}


// ------------------------------------------------------------------
plugin_manager
::plugin_manager()
  : m_impl( new plugin_manager_impl( this ) )
{
  const char * env_ptr = kwiversys::SystemTools::GetEnv(environment_variable_name  );
  if ( 0 != env_ptr )
  {
    std::string const extra_module_dirs( env_ptr );

    // Split supplied path into separate items using PATH_SEPARATOR_CHAR as delimiter
    ST::Split( extra_module_dirs, m_impl->m_search_paths, PATH_SEPARATOR_CHAR );
  }

  // Then add default paths
  ST::Split( default_module_paths, m_impl->m_search_paths, PATH_SEPARATOR_CHAR );
}


plugin_manager
::~plugin_manager()
{ }


// ------------------------------------------------------------------
plugin_factory_vector_t const&
plugin_manager
::get_factories( std::string const& type_name ) const
{
  static plugin_factory_vector_t empty; // needed for error case

  plugin_map_t::const_iterator it = m_impl->m_plugin_map.find(type_name);
  if ( it == m_impl->m_plugin_map.end() )
  {
    return empty;
  }

  return it->second;
}


// ------------------------------------------------------------------
plugin_factory_handle_t
plugin_manager
::add_factory( plugin_factory* fact )
{
  fact->add_attribute( plugin_factory::PLUGIN_FILE_NAME, m_impl->m_current_filename );

  std::string interface_type;
  fact->get_attribute( plugin_factory::INTERFACE_TYPE, interface_type );

  /// @todo make sure factory is not already in the list
  /// Check the two types as a signature.
  plugin_factory_handle_t fact_handle( fact );
  m_impl->m_plugin_map[interface_type].push_back( fact_handle );

  std::string ift;
  fact->get_attribute( plugin_factory::INTERFACE_TYPE, ift );

  std::string ct;
  fact->get_attribute( plugin_factory::CONCRETE_TYPE, ct );

  LOG_TRACE( m_impl->m_logger,
             "Adding plugin to create interface: " << ift
             << " from derived type: " << ct
             << " from file: " << m_impl->m_current_filename );

  return fact_handle;
}


// ------------------------------------------------------------------
plugin_map_t const&
plugin_manager
::get_plugin_map() const
{
  return m_impl->m_plugin_map;
}


void
plugin_manager
::add_search_path( path_t const& path)
{
  this->m_impl->m_search_paths.push_back( path );
}


std::vector< path_t > const&
plugin_manager
::get_search_path() const
{
  //return vector of paths
  return this->m_impl->m_search_paths;
}


std::vector< std::string >
plugin_manager
::get_file_list() const
{
  std::vector< std::string > retval;

  VITAL_FOREACH( plugin_manager_impl::library_map_t::value_type it, m_impl->m_library_map )
  {
    retval.push_back( it.first );
  } // end foreach

  return retval;
}


// ==================================================================
/**
 * @brief Load all known modules.
 *
 */
void
plugin_manager_impl
::load_known_modules()
{
  // Iterate over path and load modules
  VITAL_FOREACH( auto const & module_dir, m_search_paths )
  {
    look_in_directory( module_dir );
  }
}


// ------------------------------------------------------------------
void
plugin_manager_impl
::look_in_directory( path_t const& dir_path )
{
  // Check given path for validity
  // Preventing load from current directory via empty string (security)
  if ( dir_path.empty() )
  {
    LOG_DEBUG( m_logger, "Empty directory in the search path. Ignoring." );
    return;
  }
  if ( ! ST::FileExists( dir_path ) )
  {
    LOG_DEBUG( m_logger, "Path " << dir_path << " doesn't exist. Ignoring." );
    return;
  }
  if ( ! ST::FileIsDirectory( dir_path ) )
  {
    LOG_DEBUG( m_logger, "Path " << dir_path << " is not a directory. Ignoring." );
    return;
  }

  // Iterate over search-path directories, attempting module load on elements
  // that end in the configured library suffix.
  LOG_DEBUG( m_logger, "Loading modules from directory: " << dir_path );

  kwiversys::Directory dir;
  dir.Load( dir_path );
  unsigned long num_files = dir.GetNumberOfFiles();

  for (unsigned long i = 0; i < num_files; ++i )
  {
    std::string file = dir.GetPath();
    file += "/" + std::string( dir.GetFile( i ) );

    // Accept this file as a module to check if it has the correct library
    // suffix and matches a provided module name if one was provided.

    if ( ST::GetFilenameLastExtension( file ) == shared_library_suffix )
    {
      // Check that we're looking a file
      if ( ! ST::FileIsDirectory( file ) )
      {
        load_from_module( file );
      }
      else
      {
        LOG_WARN( m_logger, "Encountered a directory entry " << file <<
                  " which ends with the expected suffix, but is not a file" );
      }
    }
  } // end for
} // plugin_manager_impl::look_in_directory


// ----------------------------------------------------------------
/**
 * \brief Load single module from shared object / DLL
 *
 * @param path Name of module to load.
 */
void
plugin_manager_impl
::load_from_module( path_t const& path )
{
  DL::LibraryHandle lib_handle;

  m_current_filename = path;

  LOG_DEBUG( m_logger, "Loading plugins from: " << path );

  lib_handle = DL::OpenLibrary( path );
  if ( ! lib_handle )
  {
    std::stringstream str;
    LOG_WARN( m_logger, "plugin_manager::Unable to load shared library \""  << path << "\" : "
              << DL::LastError() );
    return;
  }

  DL::SymbolPointer fp =
    DL::GetSymbolAddress( lib_handle, "register_factories" );
  if ( 0 == fp )
  {
    std::string str("Unknown error");
    char const* last_error = DL::LastError();
    if ( last_error )
    {
      str = std::string( last_error );
    }

    LOG_WARN( m_logger, "plugin_manager:: Unable to bind to function \"register_factories()\" : "
              << last_error );

    DL::CloseLibrary( lib_handle );
    return;
  }

  // Save currently opened library in map
  m_library_map[path] = lib_handle;

  typedef void (* reg_fp_t)( plugin_manager* );

  reg_fp_t reg_fp = reinterpret_cast< reg_fp_t > ( fp );

  ( *reg_fp )( m_parent ); // register plugins
}


} } // end namespace
