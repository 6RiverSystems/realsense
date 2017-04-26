/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FILESYSTEM_HPP_
#define FILESYSTEM_HPP_

#include <string>
#include <algorithm>
using namespace std;

namespace srs {

struct Filesystem
{
    static string dirname(const string& filename)
    {
        if (filename.empty())
        {
            return "";
        }

        string::const_reverse_iterator lastSlash = find(filename.rbegin(), filename.rend(), '/');
        if (lastSlash == filename.rend())
        {
            // No slashes found
            return ".";
        }
        else if (lastSlash.base() - filename.begin() == 1)
        {
            // Slash is first char
            return "/";
        }
        else if (filename.end() == lastSlash.base())
        {
            // Slash is last char
            string redo = string(filename.begin(), filename.end() - 1);
            return Filesystem::dirname(redo);
        }

        return string(filename.begin(), lastSlash.base() - 1);
    }
};

} // namespace srs

#endif // FILESYSTEM_HPP_
