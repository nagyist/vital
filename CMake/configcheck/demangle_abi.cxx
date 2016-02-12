#include <cxxabi.h>
#include <typeinfo>
#include <string>

int main(int argc, char *argv[])
{
  std::string sym = typeid( std::string ).name();
  int status;
  char* demangled_name = abi::__cxa_demangle(sym.c_str(), NULL, NULL, &status);

  if( 0 == status )
  {
    std::free(demangled_name);
  }

  return 0;
}
