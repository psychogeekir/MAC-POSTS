#include "ults.h"

#include <string>
#include <sstream>
#include <cstdlib>
#include <fstream>

TInt MNM_Ults::round(TFlt in)
{
  TFlt rdNum = TFlt(std::rand()/(1.0*RAND_MAX));
  TFlt floorN = TFlt(TInt(in));
  if((in - floorN) >rdNum)
    return TInt(floorN + 1);
  else
    return TInt(floorN);
}

TInt MNM_Ults::min(TInt a, TInt b)
{
  return a<b? a:b;
}


TFlt MNM_Ults::min(TFlt a, TFlt b)
{
  return a<b? a:b;
}


TFlt MNM_Ults::max(TFlt a, TFlt b)
{
  return a>b? a:b;
}

TFlt MNM_Ults::divide(TFlt a, TFlt b){
  if (a == TFlt(0)) return TFlt(0);
  if (b == TFlt(0)) return TFlt(0);
  return a/b;
}

TInt MNM_Ults::mod(TInt a, TInt b){
  if (b == TInt(0)) return TInt(0);
  return a % b;
}


TFlt MNM_Ults::rand_flt(){
  return TFlt((double) rand() / (RAND_MAX));
}

TFlt MNM_Ults::max_link_cost()
{
  return TFlt(60 * 60);
}


int MNM_Ults::copy_file( const char* srce_file, const char* dest_file )
{
    std::ifstream srce( srce_file, std::ios::binary ) ;
    std::ofstream dest( dest_file, std::ios::binary ) ;
    dest << srce.rdbuf() ;
    return 0;
}


int MNM_Ults::copy_file( std::string srce_file, std::string dest_file )
{
  return MNM_Ults::copy_file(  srce_file.c_str(), dest_file.c_str() );
}

float MNM_Ults::roundoff(float value, unsigned char prec)
{
  float pow_10 = pow(10.0f, (float)prec);
  return round(value * pow_10) / pow_10;
}

bool MNM_Ults::approximate_equal(TFlt a, TFlt b, float p) 
{
  // approximately equal, https://stackoverflow.com/questions/17333/what-is-the-most-effective-way-for-float-and-double-comparison
  if (fabs(a - b) <= p * max(fabs(a), fabs(b))) {
    return true;
  }
  else {
    return false;
  }
}

bool MNM_Ults::approximate_less_than(TFlt a, TFlt b, float p) 
{
  return a + p * max(abs(a), abs(b)) < b;
}

int MNM_Ults::round_up_time(TFlt time, float p)
{
  if (time < 1) {
    return 1;
  }
  else {
    IAssert(int(time) >= 1);
    if (approximate_equal(time, TFlt(int(time)), p)) {
      return int(time);
    }
    else {
      IAssert(int(time) + 1 > time);
      return int(time) + 1;
    }
  }
}

int MNM_Ults::round_down_time(TFlt time)
{
  if (time < 1) {
    return 1;
  }
  else {
    IAssert(int(time) >= 1);
    return int(time);
  }
}

PNEGraph MNM_Ults::reverse_graph(const PNEGraph &graph)
{
  PNEGraph reversed_graph = PNEGraph::TObj::New();
  if (graph -> GetEdges() > 0) {
    int _link_ID, _from_node_ID, _to_node_ID;
    for (auto _edge_it = graph->BegEI(); _edge_it < graph->EndEI(); _edge_it++) {
      _link_ID = _edge_it.GetId();
      _from_node_ID = _edge_it.GetSrcNId();
      _to_node_ID = _edge_it.GetDstNId();
      if (! reversed_graph -> IsNode(_from_node_ID)) { reversed_graph -> AddNode(_from_node_ID); }
      if (! reversed_graph -> IsNode(_to_node_ID)) { reversed_graph -> AddNode(_to_node_ID); }
      reversed_graph -> AddEdge(_to_node_ID, _from_node_ID, _link_ID);
    }
  }
  return reversed_graph;
}


Chameleon::Chameleon(std::string const& value) 
{
    value_=value;
}

#include <iostream>

Chameleon::Chameleon(const char* c) 
{
    value_=c;
}

Chameleon::Chameleon(double d) 
{
    std::stringstream s;
    s<<d;
    value_=s.str();
}

Chameleon::Chameleon(Chameleon const& other) 
{
    value_=other.value_;
}

Chameleon& Chameleon::operator=(Chameleon const& other) 
{
    value_=other.value_;
    return *this;
}

Chameleon& Chameleon::operator=(double i) 
{
    std::stringstream s;
    s << i;
    value_ = s.str();
    return *this;
}

Chameleon& Chameleon::operator=(std::string const& s) 
{
    value_=s;
    return *this;
}

Chameleon::operator std::string() const 
{
    return value_;
}

Chameleon::operator double() const 
{
    return atof(value_.c_str());
}

std::string trim(std::string const& source, char const* delims = " \t\r\n") {
  std::string result(source);
  std::string::size_type index = result.find_last_not_of(delims);
  if(index != std::string::npos)
    result.erase(++index);

  index = result.find_first_not_of(delims);
  if(index != std::string::npos)
    result.erase(0, index);
  else
    result.erase();
  return result;
}

ConfigFile::ConfigFile(std::string const& configFile) {
  std::ifstream file(configFile.c_str());

  std::string line;
  std::string name;
  std::string value;
  std::string inSection;
  int posEqual;
  while (std::getline(file,line)) {

    if (! line.length()) continue;

    if (line[0] == '#') continue;
    if (line[0] == ';') continue;

    if (line[0] == '[') {
      inSection=trim(line.substr(1,line.find(']')-1));
      continue;
    }

    posEqual=line.find('=');
    name  = trim(line.substr(0,posEqual));
    value = trim(line.substr(posEqual+1));

    content_[inSection+'/'+name]=Chameleon(value);
  }
}

Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry) const {

  std::map<std::string,Chameleon>::const_iterator ci = content_.find(section + '/' + entry);

  if (ci == content_.end()) throw std::invalid_argument("does not exist");
  
  return ci->second;
}

Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry, double value) {
  try {
    return Value(section, entry);
  } catch(const char *) {
    return content_.insert(std::make_pair(section+'/'+entry, Chameleon(value))).first->second;
  }
}

Chameleon const& ConfigFile::Value(std::string const& section, std::string const& entry, std::string const& value) {
  try {
    return Value(section, entry);
  } catch(const char *) {
    return content_.insert(std::make_pair(section+'/'+entry, Chameleon(value))).first->second;
  }
}

MNM_ConfReader::MNM_ConfReader (const std::string& filename, std::string confKey) {
  m_configFile = new ConfigFile(filename);
  m_confKey = confKey;
}

MNM_ConfReader::~MNM_ConfReader () {
  if (m_configFile != NULL) {
    delete m_configFile;
  }
}

TInt MNM_ConfReader::get_int (const std::string& key) {
  int val = m_configFile->Value(m_confKey, key);
  return TInt(val);
}

TFlt MNM_ConfReader::get_float(const std::string& key)
{
  double val = m_configFile->Value(m_confKey, key);
  return TFlt(val);
}

std::string MNM_ConfReader::get_string(const std::string& key)
{
  std::string val = m_configFile->Value(m_confKey, key);
  return val;
}