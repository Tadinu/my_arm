/* Copyright (C) 2008, 2009 Inge Eivind Henriksen
   See the file COPYING that comes with this distribution for copying permission.
*/
/*! \file
* \brief Contains the CBase64 class headers
*/

#pragma once

#include <sstream>
#include <vector>

namespace BaseType
{
    class Base64
    {
        private:
            static const char encodeCharacterTable[];
            static const char decodeCharacterTable[];
        public:
            Base64();
            ~Base64();
            static void Encode(std::istream &in, std::ostringstream &out);
            static void Decode(std::istringstream &in, std::ostream &out);
            static bool encode64(std::string & string);
            static bool decode64(std::string & string);
            static bool encode64(std::vector<unsigned char> & data);
            static bool decode64(std::vector<unsigned char> & data);
    };
};

namespace baselib = BaseType;
