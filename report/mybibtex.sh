myarticle=$1
echo "bibtexing " $myarticle
latex $myarticle
bibtex $myarticle
latex $myarticle
latex $myarticle
