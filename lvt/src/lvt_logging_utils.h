/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Rawashdeh Research Group
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/
// Author: Mohamed Aladem

#ifndef LVT_LOGGING_UTILS_H__
#define LVT_LOGGING_UTILS_H__

#include "lvt_definitions.h"
#include "lvt_parameters.h"
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <cassert>
#include <chrono>

class lvt_log
{
  public:
    lvt_log();
    ~lvt_log();
    void init();
    void log(const std::string &str);
    void log_params(const lvt_parameters &params);

  private:
    bool m_init;
    std::ofstream m_file;
    std::chrono::high_resolution_clock::time_point m_start_time;
};

class lvt_value_recorder
{
  public:
    lvt_value_recorder();
    ~lvt_value_recorder();

    void init();
    void finish();
    void register_value(const std::string &val_name);

    void record(const std::string &val_name, int val)
    {
        assert(m_values.find(val_name) != m_values.end());
        value_holder vh;
        vh.value_type = value_holder::integer;
        vh.as_int = val;
        m_values[val_name].push_back(vh);
    }

    void record(const std::string &val_name, float val)
    {
        assert(m_values.find(val_name) != m_values.end());
        value_holder vh;
        vh.value_type = value_holder::floating_point;
        vh.as_float = val;
        m_values[val_name].push_back(vh);
    }

    void flush_frame();

  private:
    struct value_holder
    {
        enum
        {
            floating_point,
            integer
        } value_type;
        union {
            float as_float;
            int as_int;
        };
    };

    std::map<std::string, std::vector<value_holder>> m_values;
    std::vector<std::string> m_titles;
    std::ofstream m_values_file;
};

#endif //LVT_LOGGING_UTILS_H__
