#pragma once

#ifndef QPFACTORY_H
#define QPFACTORY_H

#include "QP.h"
#include "QPMosek.h"

class QPFactory
{
public:
	enum QPType { QPTYPE_BEGIN, QPMOSEKT, QPTYPE_END };

private:
	QPFactory(){ ; }

public:
	static QP* make(QPType t, bool _storeVariables = false);
	static void typeToString(QPType t, std::string& s);
};


#endif // QPFACTORY_H