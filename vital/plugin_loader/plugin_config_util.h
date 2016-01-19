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

#ifndef _VIDTK_PLUGIN_CONFIG_UTIL_H_
#define _VIDTK_PLUGIN_CONFIG_UTIL_H_

#include <string>


namespace vidtk {

  class plugin_factory;
  class config_block;

  /**
   * @brief Add config entry to plugin factory attributes
   *
   * This function sets the specified attribute that is to be later added
   * to the config block.
   *
   * @param fact Apply attribute to this foctory
   * @param name Config parameter name
   * @param description Config parameter description
   */
  void add_config_attribute( plugin_factory& fact, std::string const& name, std::string const& description );

  /**
   * @brief Add config entry to plugin factory attributes
   *
   * This function sets the specified attribute that is to be later added
   * to the config block.
   *
   * @param fact Apply attribute to this foctory
   * @param name Config parameter name
   * @param default_value Config parameter default value
   * @param description Config parameter description
   */
  void add_config_attribute( plugin_factory&    fact,
                             std::string const& name,
                             std::string const& default_value,
                             std::string const& description );

  /**
   * @brief Collect config entries from attributes and add to config block.
   *
   * This function applies all config attributes from the factory to
   * the config block. The config entry block level is taken directly
   * from the attribute key string with no other magic transforms. You
   * will have to craft that part of the entry carefully.
   *
   * @param[in] fact Plugin factory that has the attributes
   * @param[in,out] block Config block to update
   */
  void collect_plugin_config( plugin_factory& fact, config_block& block );
}

#endif /* _VIDTK_PLUGIN_CONFIG_UTIL_H_ */
