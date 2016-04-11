#include <map>
#include <memory>


int main(int argc, char *argv[])
{
  // Test that unique_ptr types can be inserted into std::map structures
  // i.e. known not to be supported in GCC 4.4.7
  typedef std::unique_ptr<int> ptr_t;
  typedef std::map< int, std::unique_ptr<int> > test_map_t;
  test_map_t test_map;

  // insert method 1
  int* x = new int( 5 );
  test_map[0] = ptr_t( x );
  // insert method 2
  int *y = new int( 100 );
  test_map.insert( test_map_t::value_type( 1, ptr_t(y) ) );

  return 0;
}
