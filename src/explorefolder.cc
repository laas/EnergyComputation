#include "explorefolder.hh"

namespace fs = boost::filesystem;
using namespace std ;

// public methods
ExploreFolder::ExploreFolder() :
  checker_ref("-rstate.log") ,
  checker_state("-astate.log")
{
}

int ExploreFolder::findLogInFolder(path_t & inputDir )
{
  if (!input_astate_files_.empty())
    input_astate_files_.clear();
  if (!input_rstate_files_.empty())
    input_rstate_files_.clear();
  recursiveParcours(inputDir);
  filterFiles();
}

// private methods
int ExploreFolder::recursiveParcours(path_t & inputDir )
{
  fs::directory_iterator end_iter;
  if ( fs::exists(inputDir) && fs::is_directory(inputDir))
  {
    for( fs::directory_iterator dir_iter(inputDir) ;
         dir_iter != end_iter ; ++dir_iter)
    {
      if (fs::is_directory(*dir_iter))
      {
        path_t path_tmp = *dir_iter ;
        recursiveParcours(path_tmp);
      }
      if (fs::is_regular_file(dir_iter->status()) )
      {
        input_rstate_files_.push_back(*dir_iter);
        input_astate_files_.push_back(*dir_iter);
      }
    }
    return 1 ;
  }
  else{
    cout << "the folder to explore doesn't exist or it is a regular file, \
            please modify the input of the application.\n" ;
            return -1 ;
  }
  return 0;
}

bool ExploreFolder::hasEnding (std::string const & fullString,
                               std::string const & ending)
{
  std::cout << "WARNING : function not implemented! undefined behavior" << std::cout;
  return false ;
}

int ExploreFolder::filterFiles()
{
  input_astate_files_.remove_if( checker_state );
  input_rstate_files_.remove_if( checker_ref );

  input_astate_files_.sort(compare_nocase);
  input_rstate_files_.sort(compare_nocase);

  result_set_t as_files = input_astate_files_ ;
  result_set_t rs_files = input_rstate_files_   ;
  input_astate_files_.clear();
  input_rstate_files_.clear();
  string astate_end("-astate.log");
  string rstate_end("-rstate.log");

  for(result_set_t::iterator as =as_files.begin() ; as!=as_files.end() ; ++as)
  {
    std::string astate, rstate ;
    astate = as->string().substr(0,as->string().size()-astate_end.size());
    for(result_set_t::iterator rs =rs_files.begin() ; rs!=rs_files.end() ; ++rs)
    {
      rstate = rs->string().substr(0,rs->string().size()-rstate_end.size());
      if (astate.compare(rstate)==0){
        input_astate_files_.push_back(*as);
        input_rstate_files_.push_back(*rs);
      }
    }
  }
  return 0 ;
}

bool compare_nocase (const path_t& first_path, const path_t& second_path)
{
  std::string first = first_path.string() ;
  std::string second = second_path.string() ;
  unsigned int i=0;
  while ( (i < first.length()) && (i < second.length()) )
  {
    if (tolower(first[i]) < tolower(second[i])) return true;
    else if (tolower(first[i]) > tolower(second[i])) return false;
    ++i;
  }
  return ( first.length() < second.length() );
}
