#ifndef _INCLUDED_POPT_PP_H_
#define _INCLUDED_POPT_PP_H_

#include <popt.h>

class POpt{
protected:
  poptContext con;
public:
  // creation and deletion
  POpt(const char *name, int argc, const char **argv,
       const poptOption *options, int flags)
    {con = poptGetContext(name,argc,argv,options,flags);}
  POpt(const char *name, int argc, char **argv,
       const poptOption *options, int flags)
    {con = poptGetContext(name,argc,(const char **)argv,options,flags);}
  ~POpt()
    {poptFreeContext(con);}

  // functions for processing options
  int getNextOpt()
    {return(poptGetNextOpt(con));}
  void ignoreOptions()
    {while(getNextOpt() >= 0);}
  const char *getOptArg()
    {return(poptGetOptArg(con));}
  const char *strError(int error)
    {return(poptStrerror(error));}
  const char *badOption(int flags = POPT_BADOPTION_NOALIAS)
    {return(poptBadOption(con,flags));}

  // processing other arguments
  const char *getArg()
    {return(poptGetArg(con));}
  void ignoreArgs()
    {while(getArg());}
};

#endif
