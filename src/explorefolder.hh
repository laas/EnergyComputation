#include <commonTools.hh>

#ifndef EXPLOREFOLDER_HH
#define EXPLOREFOLDER_HH

bool compare_nocase (const path_t& first_path, const path_t& second_path);

class Checker {
public:

    Checker(std::string name)
    {
        ending_ = name ;
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
    inline std::vector<result_set_t> * files ()
    {
        files_.clear();
        files_.push_back(files_state_input_);
        files_.push_back(files_ref_input_);
        return &files_ ;
    }

private:
    int recursiveParcours(path_t & inputDir) ;
    bool hasEnding (std::string const &fullString, std::string const &ending);
    int filterFiles();

private:
    result_set_t files_state_input_ ;
    result_set_t files_Output_ ;
    result_set_t files_ref_input_ ;

    std::vector<result_set_t> files_ ;
    Checker checker_ref ;
    Checker checker_state ;

};

#endif // EXPLOREFOLDER_HH
