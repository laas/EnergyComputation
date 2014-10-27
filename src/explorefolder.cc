#include "explorefolder.hh"

namespace fs = boost::filesystem;
using namespace std ;

// public methods
ExploreFolder::ExploreFolder()
{
}

int ExploreFolder::findLogInFolder(path_t & inputDir )
{
    if (!files_.empty())
        files_.clear();
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
                files_.push_back(*dir_iter);
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
    files_.remove_if( a_checker );
    return 0 ;
}
