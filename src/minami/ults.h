#ifndef ULTS_H
#define ULTS_H

#include "Snap.h"

#include <string>
#include <string>
#include <map>


// struct TIntHash {
//  std::size_t operator()(const TInt& i) const
//  {
//      return std::hash<int>()(i());
//  }
// };

namespace std
{
    template<> struct hash<TInt>
    {
        typedef TInt argument_type;
        typedef std::size_t result_type;
        result_type operator()(argument_type const& s) const
        {
            return std::hash<int>()(s());
        }
    };
}

class MNM_Ults
{
public:
  TInt static round(TFlt in);
  TFlt static min(TFlt a, TFlt b);
  TInt static min(TInt a, TInt b);
  TFlt static max(TFlt a, TFlt b);
  TFlt static divide(TFlt a, TFlt b);
  TInt static mod(TInt a, TInt b);
  TFlt static rand_flt();
  TFlt static max_link_cost();
  int static copy_file( const char* srce_file, const char* dest_file );
  int static copy_file( std::string srce_file, std::string dest_file );

  float static roundoff(float value, unsigned char prec);
  bool static approximate_equal(TFlt a, TFlt b, float p = 1e-4);
  bool static approximate_less_than(TFlt a, TFlt b, float p = 1e-4);

  int static round_up_time(TFlt time, float p = 1e-4);
  PNEGraph static reverse_graph(const PNEGraph &graph);
};





class Chameleon {
    public:
      Chameleon(){};
      explicit Chameleon(const std::string&);
      explicit Chameleon(double);
      explicit Chameleon(const char*);

      ~Chameleon(){};

      Chameleon(const Chameleon&);
      Chameleon& operator=(Chameleon const&);

      Chameleon& operator=(double);
      Chameleon& operator=(std::string const&);

    public:
      operator std::string() const;
      operator double     () const;
    private:
      std::string value_;
};

class ConfigFile {
  std::map<std::string,Chameleon> content_;

public:
  ConfigFile(std::string const& configFile);
  ~ConfigFile(){};

  Chameleon const& Value(std::string const& section, std::string const& entry) const;

  Chameleon const& Value(std::string const& section, std::string const& entry, double value);
  Chameleon const& Value(std::string const& section, std::string const& entry, std::string const& value);
};

class MNM_ConfReader
{
public:
  MNM_ConfReader(const std::string&, std::string);
  ~MNM_ConfReader();

  TInt get_int(const std::string&);
  std::string get_string(const std::string&);
  TFlt get_float(const std::string&);

  /* data */
  ConfigFile *m_configFile;
  std::string m_confKey;
};

#endif