#include "LPFactory.h"


LinearP *LPFactory::make(LPFactory::LPType t, bool _storeVariables)
{
	LinearP* lp = NULL;

	switch (t){

	case LPTYPE_BEGIN:
		return lp;

	case LPTYPE_END:
		return lp;

	case LPMOSEKT:
		lp = new LPMosek();
		break;

	}

	lp->setStoreVariables(_storeVariables);
	return lp;
}

void LPFactory::typeToString(LPFactory::LPType t, std::string &s)
{
	switch (t){
	case LPTYPE_BEGIN:	s = std::string("Invalid LP Type");
		break;
	case LPTYPE_END:	s = std::string("Invalid LP Type");
		break;
		break;
	case LPMOSEKT:		s = std::string("LPMosek");
		break;
	}
}
