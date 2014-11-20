#include "explorefolder.hh"

namespace fs = boost::filesystem;
using namespace std ;

// public methods
ExploreFolder::ExploreFolder() : checker_ref("-rstate.log") , checker_state("-astate.log")
{
}

int ExploreFolder::findLogInFolder(path_t & inputDir )
{
    if (!files_state_input_.empty())
        files_state_input_.clear();
    if (!files_ref_input_.empty())
        files_ref_input_.clear();
    recursiveParcours(inputDir);
    filterFiles();
}

// private methods
int ExploreFolder::recursiveParcours(path_t & inputDir )
{
    fs::directory_iterator end_iter;
    if ( fs::exists(inputDir) && fs::is_directory(inputDir))
    {
        for( fs::directory_iterator dir_iter(inputDir) ; dir_iter != end_iter ; ++dir_iter)
        {
            if (fs::is_directory(*dir_iter))
            {
                path_t path_tmp = *dir_iter ;
                recursiveParcours(path_tmp);
            }
            if (fs::is_regular_file(dir_iter->status()) )
            {
                files_ref_input_.push_back(*dir_iter);
                files_state_input_.push_back(*dir_iter);
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

bool ExploreFolder::hasEnding (std::string const & fullString, std::string const & ending)
{

}

int ExploreFolder::filterFiles()
{
    files_state_input_.remove_if( checker_state );
    files_ref_input_.remove_if( checker_ref );

    files_state_input_.sort(compare_nocase);
    files_ref_input_.sort(compare_nocase);

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
