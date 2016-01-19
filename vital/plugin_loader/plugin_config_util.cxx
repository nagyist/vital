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

#include "plugin_config_util.h"

#include "plugin_factory.h"
#include <vital/config/config_block.h>

#include <vital/logger/logger.h>

namespace kwiver {
namespace vital {

// Special config attribute prefix. This is used to indicate that an
// attribute is destined to be added to the config.
const std::string CONFIG_PREFIX( "(config)" );


// ------------------------------------------------------------------
void add_config_attribute( plugin_factory& fact,
                           std::string const& attr,
                           std::string const& description )
{
  fact.add_attribute( CONFIG_PREFIX + attr, description );
}


// ------------------------------------------------------------------
void add_config_attribute( plugin_factory& fact,
                           std::string const& attr,
                           std::string const& def_val,
                           std::string const& description )
{
  std::string val;
  val = def_val + '\004' + description; // insert separator in string
  fact.add_attribute( CONFIG_PREFIX + attr, val );
}


// ------------------------------------------------------------------

struct update_config_functor
{
  update_config_functor( config_block& block )
    : m_block( block )
  { }

  // There is a semantic issue here when we are collecting config
  // entries.  In some polymorphic cases, all leaf nodes may
  // legitimately register the same config entry (such as
  // filename). In this case subsequent appearances of the same config
  // key should overwrite or not be added a second time.
  //
  // This brings up what to do when both (or all) config entries with
  // the same key have different defaults.
  // - Remove default from entry and force configuration from the file.
  // - Log an error/warning and terminate
  // - modify our API and not allow default values int the config attribs.

  void operator()( std::string const& key, std::string const& val )
  {
    // If the key has the super secret config prefix, remove that
    // prefix and add the rest to the config.
    if (key.find( CONFIG_PREFIX ) == 0 )
    {
      const std::string name = key.substr( CONFIG_PREFIX.size() ) ;
      try
      {
        // See if we have the two parameter version or the three?
        // If the \004 marker appears in the string anywhere, we have 3
        size_t idx =  val.find( '\004' );
        if ( std::string::npos != idx )
        {
          // If the marker happens to be the first element in the string,
          // we have an empty string as the default, let that pass through.
          // Otherwise, get the substring that represents the default value.
          std::string def_val = "";
          if ( 0 != idx )
          {
            def_val = val.substr( 0, idx );
          }

          const std::string descrip = val.substr( idx+1 );

          // add with description
          m_block.set_value( name, def_val, descrip );
        }
        else
        {
          // Add without description
          m_block.set_value( name, val );
        }
      }
      catch (std::runtime_error const& e)
      {
        static kwiver::vital::logger_handle_t logger( kwiver::vital::get_logger( "vital.plugin_config_util" ) );
        LOG_WARN( logger, "Exception caught merging configs: " << e.what() );
      }
    }
  }

  // Member data
  config_block& m_block;
};


// ------------------------------------------------------------------
void collect_plugin_config( plugin_factory& fact, config_block& block )
{
  update_config_functor ucf( block );
  fact.for_each_attr( ucf );
}

} } // end namespace
