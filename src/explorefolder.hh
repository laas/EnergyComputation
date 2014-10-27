#include <commonTools.hh>

#ifndef EXPLOREFOLDER_HH
#define EXPLOREFOLDER_HH

class Checker {
public:

    Checker()
    {
        ending_ = "-astate.log" ;
        endLength_ = ending_.length() ;
    }

    bool operator()(const path_t fullString)
    {
        std::string inputString = fullString.string() ;
        int inLength = inputString.length() ;

        if (inLength >= endLength_) {
            return (0 != inputString.compare (inLength - endLength_, endLength_, ending_));
        } else {
            return false;
        }
    }

    void setEnding(std::string a_end)
    {
       ending_ = a_end ;
       endLength_ = ending_.length() ;
    }

private :
    std::string ending_ ;
    int endLength_ ;
};

class ExploreFolder
{
public:
    ExploreFolder();

    // explore a folder to get the names of the files recursively
    int findLogInFolder(path_t & inputDir) ;

    // check if the file is the file is a OpenHRP _astate.log file
    inline bool is_not_astate_log (const path_t & input_path)
        { return ExploreFolder::hasEnding (input_path.string(), "-astate.log"); }

    // getter
    inline result_set_t * files ()
    {return &files_ ;}

private:
    int recursiveParcours(path_t & inputDir) ;
    bool hasEnding (std::string const &fullString, std::string const &ending);
    int filterFiles();

private:
    result_set_t files_ ;
    result_set_t files_Output_ ;
    Checker a_checker ;

};

#endif // EXPLOREFOLDER_HH
