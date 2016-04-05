/*
 * IO.h
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */
#include <functional>

#ifndef SRC_IO_H_
#define SRC_IO_H_

namespace srs {

class IO {
public:

	virtual ~IO( ) { };

	virtual void Open( const char* pszName, std::function<void(std::vector<char>)> readCallback ) = 0;

	virtual bool IsOpen( ) const = 0;

	virtual void Close( ) = 0;

	virtual void Write( std::vector<char> buffer ) = 0;

};

} /* namespace srs */

#endif /* SRC_IO_H_ */
