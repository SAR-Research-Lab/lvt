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

#ifndef LVT_DEFINITIONS_H__
#define LVT_DEFINITIONS_H__

#define LVT_REPROJECTION_TH2 5.991
#define LVT_N_MAP_POINTS 1000
#define LVT_ROW_MATCHING_VERTICAL_SEARCH_RADIUS 2
#define LVT_HASHING_CELL_SIZE 25
#define LVT_CORNERS_LOW_TH 200
#define LVT_N_MATCHES_TH 50

// Comment/uncomment to disable/enable recording the measurments for post-processing
//#define LVT_ENABLE_MEASURMENT_RECORD
#define LVT_ENABLE_LOG
//#define LVT_ENABLE_VISUALIZATION

#ifndef LVT_ENABLE_LOG
#define LVT_ASSERT(expr) ((void)0)
#define LVT_ADD_LOGGING
#define LVT_LOG(str) ((void)0)
#else
#define LVT_ASSERT(expr)                                                                                                                                                                     \
    if (!!(expr))                                                                                                                                                                            \
        ;                                                                                                                                                                                    \
    else                                                                                                                                                                                     \
    {                                                                                                                                                                                        \
        _the_log->log(std::string("Assertion failed in: ") + std::string(__FILE__) + std::string(" Line #") + std::to_string(__LINE__) + std::string(" Expression: ") + std::string(#expr)); \
        assert(0);                                                                                                                                                                           \
    }

#define LVT_ADD_LOGGING      \
    friend class lvt_system; \
    class lvt_log *_the_log;

#define LVT_LOG(str) _the_log->log(str);
#endif // LVT_ENABLE_LOG

#ifndef LVT_ENABLE_MEASURMENT_RECORD
#define LVT_ADD_MEASURMENTS_RECORDER
#define LVT_RECORD(value_name, val) ((void)0)
#define LVT_FLUSH_RECORD() ((void)0)
#else
#define LVT_ADD_MEASURMENTS_RECORDER class lvt_value_recorder *_the_value_recorder;
#define LVT_RECORD(value_name, val) _the_value_recorder->record(value_name, val);
#define LVT_FLUSH_RECORD() _the_value_recorder->flush_frame();
#endif //LVT_ENABLE_MEASURMENT_RECORD

#endif //LVT_DEFINITIONS_H__
