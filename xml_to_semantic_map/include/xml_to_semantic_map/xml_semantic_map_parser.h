#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>
#include "StringTokenizer.h"

// The increment between the OC base types
#define INCREMENT_OC 1000
#define OC_FLOOR   1000
#define OC_CEILING 2000
//#define OC_WALL    3000
#define OC_WALL_X  4000
#define OC_WALL_Y  5000
#define OC_TABLE   6000
#define OC_HANDLE             17000
#define OC_KNOB               18000
#define OC_CONTAINER          20000
#define OC_UNDERSEGMENTED     21000
#define OC_DISHASHER          22000
#define OC_OVEN               23000
#define OC_DRAWER             24000
#define OC_CUPBOARD           25000
//#define OC_CABINET          25000
#define OC_CLOSET             26000

std::string getTypeName (int type)
{
  switch (type)
  {
    case OC_CONTAINER:      return "container"; break;
    case OC_UNDERSEGMENTED: return "undersegmented"; break;
    case OC_DISHASHER:      return "dishwasher"; break;
    case OC_OVEN:           return "oven"; break;
    case OC_DRAWER:         return "drawer"; break;
    case OC_CUPBOARD:       return "cupboard"; break;
    //case OC_CABINET:        return "cabinet"; break;
    case OC_CLOSET:         return "closet"; break;
    case OC_FLOOR:          return "floor"; break;
    case OC_CEILING:        return "ceiling"; break;
    //case OC_WALL:           return "wall"; break;
    case OC_WALL_X:         return "wall_x"; break;
    case OC_WALL_Y:         return "wall_y"; break;
    case OC_TABLE:          return "horizontal"; break;
    default:;
  }
  return "UNDEFINED";
}

inline int
  checkObjectClass (double ocDim)
{
  return (((int)(ocDim) / INCREMENT_OC) * INCREMENT_OC);
}

inline int
  getObjectClassIndex (double ocDim)
{
  return ((int)(ocDim) % INCREMENT_OC);
}

inline void
  getObjectClassAndIndex (double ocDim, int &oc, int &idx)
{
  oc = (((int)(ocDim) / INCREMENT_OC) * INCREMENT_OC);
  idx = ((int)(ocDim) % INCREMENT_OC);
}

inline int
  getObjectClassDimension (int oc, int idx)
{
  return oc + idx;
}

struct Plane
{
  std::string name;
  int id;

  double model[4];      // plane equation (4)
  double minD[3];       // minimum dimensions
  double maxD[3];       // maximum dimensions

  bool
    operator < (Plane const & b) const
  {
    return this->id < b.id;
  }

  bool
    operator > (Plane const & b) const
  {
    return this->id > b.id;
  }

  bool
    operator == (Plane const & b) const
  {
    return this->id == b.id;
  }
};

struct Knob
{
  std::string name;
  int id;

  double center[3];     // center of this knob (3)
  double radius;        // radius of knob circle
  double doorID;        // door ID (OC number)

  bool
    operator < (Knob const & b) const
  {
    return this->id < b.id;
  }

  bool
    operator > (Knob const & b) const
  {
    return this->id > b.id;
  }

  bool
    operator == (Knob const & b) const
  {
    return this->id == b.id;
  }
};

struct Handle
{
  std::string name;
  int id;

  double center[3];     // center of this knob (3)
  double elongation[3]; // elongation (3d) of the handle
  double doorID;        // door ID (OC number)

  bool
    operator < (Handle const & b) const
  {
    return this->id < b.id;
  }

  bool
    operator > (Handle const & b) const
  {
    return this->id > b.id;
  }

  bool
    operator == (Handle const & b) const
  {
    return this->id == b.id;
  }
};

struct Face
{
  std::string side;
  double p0[3], p1[3], p2[3], p3[3];
};

struct Candidate
{
  std::string name;
  int id, type;
  int wallID;

  Face front, back;

  std::vector<int> handles;
  std::vector<int> knobs;

  bool
    operator < (Candidate const & b) const
  {
    return this->id < b.id;
  }

  bool
    operator > (Candidate const & b) const
  {
    return this->id > b.id;
  }

  bool
    operator == (Candidate const & b) const
  {
    return this->id == b.id;
  }
};

struct SemanticMap
{
  std::vector<Candidate> candidates;
  std::vector<Knob> knobs;
  std::vector<Handle> handles;
  std::vector<Plane> planes;
};

//SemanticMap smap;
std::string interpretNode (const xmlpp::Node* node, SemanticMap &smap);

////////////////////////////////////////////////////////////////////////////////
void
  ParseModel (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Plane w;
  StringTokenizer st = StringTokenizer (textValue, " ");
  w.model[0] = st.nextFloatToken ();
  w.model[1] = st.nextFloatToken ();
  w.model[2] = st.nextFloatToken ();
  w.model[3] = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    w.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Plane>::iterator it = find (smap.planes.begin (), smap.planes.end (), w);
  if (it != smap.planes.end ())
  {
    Plane *pwall = &(*it);
    for (int d = 0; d < 4; d++)
      pwall->model[d] = w.model[d];
#if DEBUG
    fprintf (stderr, "Adding [%s:model:%d:<%g %g %g %g>] to the map.\n", pwall.name.c_str (), w.id, w.model[0], w.model[1], w.model[2], w.model[3]);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseMinD (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Plane w;
  StringTokenizer st = StringTokenizer (textValue, " ");
  w.minD[0] = st.nextFloatToken ();
  w.minD[1] = st.nextFloatToken ();
  w.minD[2] = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    w.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Plane>::iterator it = find (smap.planes.begin (), smap.planes.end (), w);
  if (it != smap.planes.end ())
  {
    Plane *pwall = &(*it);
    for (int d = 0; d < 3; d++)
      pwall->minD[d] = w.minD[d];
#if DEBUG
    fprintf (stderr, "Adding [%s:minD:%d:<%g %g %g>] to the map.\n", pwall.name.c_str (), w.id, w.minD[0], w.minD[1], w.minD[2]);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseMaxD (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Plane w;
  StringTokenizer st = StringTokenizer (textValue, " ");
  w.maxD[0] = st.nextFloatToken ();
  w.maxD[1] = st.nextFloatToken ();
  w.maxD[2] = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    w.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Plane>::iterator it = find (smap.planes.begin (), smap.planes.end (), w);
  if (it != smap.planes.end ())
  {
    Plane *pwall = &(*it);
    for (int d = 0; d < 3; d++)
      pwall->maxD[d] = w.maxD[d];
#if DEBUG
    fprintf (stderr, "Adding [%s:maxD:%d:<%g %g %g>] to the map.\n", pwall.name.c_str (), w.id, w.maxD[0], w.maxD[1], w.maxD[2]);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseCenter (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Knob k;
  Handle h;
  StringTokenizer st = StringTokenizer (textValue, " ");
  k.center[0] = h.center[0] = st.nextFloatToken ();
  k.center[1] = h.center[1] = st.nextFloatToken ();
  k.center[2] = h.center[2] = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    k.id = h.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Knob>::iterator it = find (smap.knobs.begin (), smap.knobs.end (), k);
  if (it != smap.knobs.end ())
  {
    Knob *pknob = &(*it);
    for (int d = 0; d < 3; d++)
      pknob->center[d] = k.center[d];
#if DEBUG
    fprintf (stderr, "Adding [%s:center:%d:<%g %g %g>] to the map.\n", pknob.name.c_str (), k.id, k.center[0], k.center[1], k.center[2]);
#endif
  }
  // No knobs found with this ID
  else
  {
    std::vector<Handle>::iterator it = find (smap.handles.begin (), smap.handles.end (), h);
    if (it != smap.handles.end ())
    {
      Handle *phandle = &(*it);
      for (int d = 0; d < 3; d++)
        phandle->center[d] = h.center[d];
#if DEBUG
      fprintf (stderr, "Adding [%s:center:%d:<%g %g %g>] to the map.\n", phandle.name.c_str (), h.id, h.center[0], h.center[1], h.center[2]);
#endif
    }
    else
    {
      fprintf (stderr, "[ParseCenter] Neither handle or knob found as parent for this <center>!\n");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseRadius (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Knob k;
  StringTokenizer st = StringTokenizer (textValue, " ");
  k.radius = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    k.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Knob>::iterator it = find (smap.knobs.begin (), smap.knobs.end (), k);
  if (it != smap.knobs.end ())
  {
    Knob *pknob = &(*it);
    pknob->radius = k.radius;
#if DEBUG
    fprintf (stderr, "Adding [%s:radius:%d:<%g>] to the map.\n", pknob.name.c_str (), k.id, k.radius);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseDoorID (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Knob k;
  Handle h;
  StringTokenizer st = StringTokenizer (textValue, " ");
  k.doorID = h.doorID = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    k.id = h.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Knob>::iterator it = find (smap.knobs.begin (), smap.knobs.end (), k);
  if (it != smap.knobs.end ())
  {
    Knob *pknob = &(*it);
    pknob->doorID = k.doorID;
#if DEBUG
    fprintf (stderr, "Adding [%s:doorID:%d:<%g>] to the map.\n", pknob.name.c_str (), k.id, k.doorID);
#endif
  }
  // No knobs found with this ID
  else
  {
    std::vector<Handle>::iterator it = find (smap.handles.begin (), smap.handles.end (), h);
    if (it != smap.handles.end ())
    {
      Handle *phandle = &(*it);
      phandle->doorID = h.doorID;
#if DEBUG
      fprintf (stderr, "Adding [%s:doorID:%d:<%g>] to the map.\n", phandle.name.c_str (), h.id, h.doorID);
#endif
    }
    else
    {
      fprintf (stderr, "[ParseDoorID] Neither handle or knob found as parent for this <center>!\n");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseElongation (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Handle h;
  StringTokenizer st = StringTokenizer (textValue, " ");
  h.elongation[0] = st.nextFloatToken ();
  h.elongation[1] = st.nextFloatToken ();
  h.elongation[2] = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    h.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Handle>::iterator it = find (smap.handles.begin (), smap.handles.end (), h);
  if (it != smap.handles.end ())
  {
    Handle *phandle = &(*it);
    for (int d = 0; d < 3; d++)
      phandle->elongation[d] = h.elongation[d];
#if DEBUG
    fprintf (stderr, "Adding [%s:elongation:%d:<%g %g %g>] to the map.\n", phandle.name.c_str (), h.id, h.elongation[0], h.elongation[1], h.elongation[2]);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseWallID (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Candidate c;
  StringTokenizer st = StringTokenizer (textValue, " ");
  c.wallID = st.nextFloatToken ();

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    c.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Candidate>::iterator it = find (smap.candidates.begin (), smap.candidates.end (), c);
  if (it != smap.candidates.end ())
  {
    Candidate *pcandidate = &(*it);
    pcandidate->wallID = c.wallID;
#if DEBUG
    fprintf (stderr, "Adding [%s:wallID:%d:<%d>] to the map.\n", pcandidate.name.c_str (), c.id, c.wallID);
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseHandles (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Candidate c;
  StringTokenizer st = StringTokenizer (textValue, " ");
  while (st.hasMoreTokens ())
  {
    c.handles.push_back (st.nextIntToken ());
  }

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    c.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Candidate>::iterator it = find (smap.candidates.begin (), smap.candidates.end (), c);
  if (it != smap.candidates.end ())
  {
    Candidate *pcandidate = &(*it);

#if DEBUG
    fprintf (stderr, "Adding [%s:handles:%d:< ", pcandidate.name.c_str (), c.id);
#endif

    for (unsigned int d = 0; d < c.handles.size (); d++)
    {
      pcandidate->handles.push_back (c.handles.at (d));
#if DEBUG
      print_value (stderr, "%d ", c.handles.at (d));
#endif
    }
#if DEBUG
    fprintf (stderr, ">] to the map.\n");
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseKnobs (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;
  xmlpp::Node::NodeList::iterator lit = list.begin ();
  std::string textValue = interpretNode (*lit, smap);

  Candidate c;
  StringTokenizer st = StringTokenizer (textValue, " ");
  while (st.hasMoreTokens ())
  {
    c.knobs.push_back (st.nextIntToken ());
  }

  const xmlpp::Element *parent = node->get_parent ();

  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    c.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Candidate>::iterator it = find (smap.candidates.begin (), smap.candidates.end (), c);
  if (it != smap.candidates.end ())
  {
    Candidate *pcandidate = &(*it);

#if DEBUG
    fprintf (stderr, "Adding [%s:knobs:%d:< ", pcandidate.name.c_str (), c.id);
#endif

    for (unsigned int d = 0; d < c.knobs.size (); d++)
    {
      pcandidate->knobs.push_back (c.knobs.at (d));
#if DEBUG
      print_value (stderr, "%d ", c.knobs.at (d));
#endif
    }
#if DEBUG
    fprintf (stderr, ">] to the map.\n");
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void
  ParseFace (const xmlpp::Node* node, SemanticMap &smap)
{
  // Recurse through child nodes to get their values
  xmlpp::Node::NodeList list = node->get_children ();
  if (list.size () == 0)
    return;

  // Get the face type (front|back)
  const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node);
  const xmlpp::Attribute* sideAttr = nodeElement->get_attribute ("side");

  if (!sideAttr)
    return;

  Face f;
  f.side = sideAttr->get_value ();

  for (xmlpp::Node::NodeList::iterator lit = list.begin (); lit != list.end (); ++lit)
  {
    Glib::ustring nodeName = (*lit)->get_name ();
    const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*lit);

    if (!nodeText)
    {
      xmlpp::Node::NodeList list_face = (*lit)->get_children ();

      xmlpp::Node::NodeList::iterator lit_face = list_face.begin ();
      std::string textValue = interpretNode (*lit_face, smap);

      // Tokenize the values
      StringTokenizer st = StringTokenizer (textValue, " ");

      // Get the name of the parent
      const xmlpp::Element *parent = (*lit_face)->get_parent ();
      if (parent->get_name () == "point0")
      {
        f.p0[0] = st.nextFloatToken ();
        f.p0[1] = st.nextFloatToken ();
        f.p0[2] = st.nextFloatToken ();
      }
      if (parent->get_name () == "point1")
      {
        f.p1[0] = st.nextFloatToken ();
        f.p1[1] = st.nextFloatToken ();
        f.p1[2] = st.nextFloatToken ();
      }
      if (parent->get_name () == "point2")
      {
        f.p2[0] = st.nextFloatToken ();
        f.p2[1] = st.nextFloatToken ();
        f.p2[2] = st.nextFloatToken ();
      }
      if (parent->get_name () == "point3")
      {
        f.p3[0] = st.nextFloatToken ();
        f.p3[1] = st.nextFloatToken ();
        f.p3[2] = st.nextFloatToken ();
      }
    }
  }


  // Get this face's parent
  const xmlpp::Element *parent = node->get_parent ();

  Candidate c;
  const xmlpp::Attribute* regionNrAttr = parent->get_attribute ("id");
  if (regionNrAttr)
  {
    std::string s = regionNrAttr->get_value ();
    c.id = atoi (s.c_str ());
  }
  else
    fprintf (stderr, "No ID attribute found.\n");

  std::vector<Candidate>::iterator it = find (smap.candidates.begin (), smap.candidates.end (), c);
  if (it != smap.candidates.end ())
  {
    Candidate *pcandidate = &(*it);
    // Copy this face information
    if (f.side == "front")
    {
      pcandidate->front.side = f.side;
      for (int d = 0; d < 3; d++)
      {
        pcandidate->front.p0[d] = f.p0[d];
        pcandidate->front.p1[d] = f.p1[d];
        pcandidate->front.p2[d] = f.p2[d];
        pcandidate->front.p3[d] = f.p3[d];
      }
    }
    else if (f.side == "back")
    {
      pcandidate->back.side = f.side;
      for (int d = 0; d < 3; d++)
      {
        pcandidate->back.p0[d] = f.p0[d];
        pcandidate->back.p1[d] = f.p1[d];
        pcandidate->back.p2[d] = f.p2[d];
        pcandidate->back.p3[d] = f.p3[d];
      }
    }
#if DEBUG
    fprintf (stderr, "Adding [%s:face:%s] to the map.\n", pcandidate.name.c_str (), f.side.c_str ());
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string
  interpretNode (const xmlpp::Node* node, SemanticMap &smap)
{
  const xmlpp::ContentNode* nodeContent = dynamic_cast<const xmlpp::ContentNode*>(node);
  const xmlpp::TextNode* nodeText       = dynamic_cast<const xmlpp::TextNode*>(node);

  std::string regionValue, nameValue, typeValue;
  if (nodeText && nodeText->is_white_space ()) //Let's ignore the indenting - you don't always want to do this.
    return std::string ("");

  Glib::ustring nodeName = node->get_name ();

  // Treat the various node types differently:
  if (nodeText)
  {
    std::string textValue = std::string (nodeText->get_content ());     // PCDATA
    return textValue;
  }
  else if (const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node))
  {
    const xmlpp::Attribute* regionNrAttr = nodeElement->get_attribute ("id");
    const xmlpp::Attribute* nameAttr     = nodeElement->get_attribute ("name");
    const xmlpp::Attribute* typeAttr     = nodeElement->get_attribute ("type");
    if (regionNrAttr)
      regionValue = regionNrAttr->get_value ();
    if (nameAttr)
       nameValue = std::string (nameAttr->get_value ());
    if (typeAttr)
       typeValue = std::string (typeAttr->get_value ());
  }

  /// Process the node just stored
  if (nodeName == "wall" || nodeName == "horizontal")
  {
    Plane w;
    w.name = nameValue;
    w.id = atoi (regionValue.c_str ());

    smap.planes.push_back (w);
#if DEBUG
    fprintf (stderr, "Adding node [%s:%d] to the map.\n", w.name.c_str (), w.id);
#endif
  }
  else if (nodeName == "knob")
  {
    Knob k;
    k.name = nameValue;
    k.id = atoi (regionValue.c_str ());

    smap.knobs.push_back (k);
#if DEBUG
    fprintf (stderr, "Adding node [%s:%d] to the map.\n", k.name.c_str (), k.id);
#endif
  }
  else if (nodeName == "handle")
  {
    Handle h;
    h.name = nameValue;
    h.id = atoi (regionValue.c_str ());

    smap.handles.push_back (h);
#if DEBUG
    fprintf (stderr, "Adding node [%s:%d] to the map.\n", h.name.c_str (), h.id);
#endif
  }
  else if (nodeName == "candidate")
  {
    Candidate c;
    c.name = nameValue;
    c.type = atoi (typeValue.c_str ());
    c.id = atoi (regionValue.c_str ());

    smap.candidates.push_back (c);
#if DEBUG
    fprintf (stderr, "Adding node [%s:%d:%d] to the map.\n", c.name.c_str (), c.id, c.type);
#endif
  }
  else if (nodeName == "model")
    ParseModel (node, smap);
  else if (nodeName == "minD")
    ParseMinD (node, smap);
  else if (nodeName == "maxD")
    ParseMaxD (node, smap);
  else if (nodeName == "center")
    ParseCenter (node, smap);
  else if (nodeName == "radius")
    ParseRadius (node, smap);
  else if (nodeName == "doorID")
    ParseDoorID (node, smap);
  else if (nodeName == "elongation")
    ParseElongation (node, smap);
  else if (nodeName == "wallID")
    ParseWallID (node, smap);
  else if (nodeName == "handles")
    ParseHandles (node, smap);
   else if (nodeName == "knobs")
     ParseKnobs (node, smap);
   else if (nodeName == "face")
     ParseFace (node, smap);

  if (!nodeContent)
  {
    // Recurse through child nodes:
    xmlpp::Node::NodeList list = node->get_children ();
    for (xmlpp::Node::NodeList::iterator iter = list.begin (); iter != list.end (); ++iter)
    {
      interpretNode (*iter, smap); //recursive
    }
  }

  return std::string ("");
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool
  parseXML (char* filename, SemanticMap &smap)
{
  bool success = false;
  try
  {
    xmlpp::DomParser parser;
    parser.set_validate ();
    parser.set_substitute_entities (); //We just want the text to be resolved/unescaped automatically.
    parser.parse_file (filename);
    if (parser)
    {
      // Walk the tree
      const xmlpp::Node* pNode = parser.get_document ()->get_root_node (); //deleted by DomParser.
      interpretNode (pNode, smap);
      success = true;
    }
  }
  catch (const std::exception& ex)
  {
    fprintf (stderr, "Exception caught: %s\n", ex.what ());
  }
  return success;
}

////////////////////////////////////////////////////////////////////////////////
// Get the minimum and maximum coordinates of a cuboid Candidate in 3D
int
  getMinMaxCandidate (Candidate c, double *minP, double *maxP)
{
  for (int d = 0; d < 3; d++)
  {
    minP[d] = FLT_MAX;
    maxP[d] = -FLT_MAX;
  }

  double center[3] = {0, 0, 0};
  for (int d = 0; d < 3; d++)
  {
    center[d] += c.front.p0[d];
    center[d] += c.front.p1[d];
    center[d] += c.front.p2[d];
    center[d] += c.front.p3[d];
  }
  for (int d = 0; d < 3; d++)
    center[d] /= 4;

  double minVal = FLT_MAX;
  int minIdx = -1;
  for (int d = 0; d < 3; d++)
  {
    if (c.front.p0[d] < minP[d]) minP[d] = c.front.p0[d];
    if (c.front.p1[d] < minP[d]) minP[d] = c.front.p1[d];
    if (c.front.p2[d] < minP[d]) minP[d] = c.front.p2[d];
    if (c.front.p3[d] < minP[d]) minP[d] = c.front.p3[d];

    if (c.front.p0[d] > maxP[d]) maxP[d] = c.front.p0[d];
    if (c.front.p1[d] > maxP[d]) maxP[d] = c.front.p1[d];
    if (c.front.p2[d] > maxP[d]) maxP[d] = c.front.p2[d];
    if (c.front.p3[d] > maxP[d]) maxP[d] = c.front.p3[d];

    if (fabs (center[d] - minP[d]) < minVal)
    {
      minVal = fabs (maxP[d] - minP[d]);
      minIdx = d;
    }
  }

  return minIdx;
}

