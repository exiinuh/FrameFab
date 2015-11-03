#ifndef LPFACTORY_H
#define LPFACTORY_H

#include "LPMosek.h"

class LPFactory
{
public:
	enum LPType { LPTYPE_BEGIN, LPMOSEKT, LPTYPE_END };

private:
	LPFactory(){ ; }

public:
	static LinearP* make(LPType t, bool _storeVariables = false);
	static void typeToString(LPType t, std::string& s);
};


#endif // LPFACTORY_H
