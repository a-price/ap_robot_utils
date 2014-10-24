/**
 *
 * \file SimpleCSVReader.h
 * \brief Simple class for reading in a simple csv file (no escaped \n's)
 *
 * Derived from example at http://stackoverflow.com/questions/1120140/csv-parser-in-c
 *
 * \author Andrew Price
 * \date May 30, 2013
 *
 * \copyright
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SIMPLECSVREADER_H_
#define SIMPLECSVREADER_H_

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace ap
{

class CSVRow
{
public:
	CSVRow(char delimiter = ',')
	{
		m_delimiter = delimiter;
	}

	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string         line;
		std::getline(str,line);

		std::stringstream   lineStream(line);
		std::string         cell;

		m_data.clear();
		while(std::getline(lineStream,cell,m_delimiter))
		{
			m_data.push_back(cell);
		}
	}
private:
	std::vector<std::string>    m_data;
	char m_delimiter;
};

std::istream& operator>>(std::istream& str,CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

class SimpleCSVReader
{
public:
	SimpleCSVReader(std::string& filename, bool firstLineHeaders = true, char delimiter = ',');
	virtual ~SimpleCSVReader() {}

	CSVRow headers;
	std::vector<CSVRow> data;
};

} /* namespace ap */
#endif /* SIMPLECSVREADER_H_ */
