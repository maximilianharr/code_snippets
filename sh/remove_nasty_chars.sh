# @brief This script removes nasty characters for Ubuntu
# @todo loop over nasty characters

# folders
find -name "* *" -type d | rename 's/ /_/g'
find -name "*-*" -type d | rename 's/-/_/g'
find -name "*(*" -type d | rename 's/\(//g'
find -name "*)*" -type d | rename 's/\)//g'

# files
find -name "*)*" -type f | rename 's/\)//g'
find -name "*(*" -type f | rename 's/\(//g'
find -name "* *" -type f | rename 's/ /_/g'
find -name "*,*" -type f | rename 's/,//g'
