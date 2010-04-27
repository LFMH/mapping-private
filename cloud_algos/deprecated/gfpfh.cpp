#include "../common/ANN-VTK.h"
#include "../common/CommonIORoutines.h"
#include "../common/CommonANNRoutines.h"
#include "../common/CommonTerminalRoutines.h"
#include "../common/CommonVTKRoutines.h"
#include "../common/VTKShapes.h"

#include "../common/octree/leveloctree.h"
#include "../common/octree/octree-ann.h"
#include "../common/octree/octree-vtk.h"
#include "../common/octree/linear.h"

#define LEAF_WIDTH 0.01  // 1cm

#define GUI 0
#define DEBUG_GUI 0

#define EMPTY_VALUE -1

using namespace cANN;
using namespace std;

struct LeafStruc
{
  double distance;
  int x, y, z;
  int nr_points;
};

inline bool
  histogramElementCompare (const pair<double, LeafStruc> &p1, const pair<double, LeafStruc> &p2)
{
  return (p1.second.distance < p2.second.distance);
}

inline double
  getDominantElement (ANNpointArray points, vector<int> *indices, int idx, vector<vector<int> > *regions)
{
/*  multiset<int> classes;
  for (unsigned int i = 0; i < indices->size (); i++)
    classes.insert (points[indices->at (i)][idx]);
  
  int max_cnt = 0;
  double max_val = -1;
  for (set<int>::iterator i = classes.begin (); i != classes.end (); ++i)
  {
    if ((int)classes.count (*i) > max_cnt)
    {
      max_cnt = classes.count (*i);
      max_val = *i;
    }
  }*/
  
  vector<int> counter (regions->size ());
  for (unsigned int i = 0; i < indices->size (); i++)
  {
    for (unsigned int d = 0; d < regions->size (); d++)
    {
      if ((int)points[indices->at (i)][idx] == points[regions->at (d)[0]][idx])
        counter[d]++;
    }
  }
  
  int max_cnt = 0;
  double max_val = -1;
  for (unsigned int i = 0; i < counter.size (); i++)
  {
    if (counter[i] > max_cnt)
    {
      max_cnt = counter[i];
      max_val = points[regions->at (i)[0]][idx];
    }
  }
  return (max_val);
}

////////////////////////////////////////////////////////////////////////////////
// Bounding box intersection modified from Graphics Gems Vol I. The method
// returns a non-zero value if the bounding box is hit. line_origin[3] starts
// the ray, line_dir[3] is the vector components of the ray in the x-y-z
// directions.
/// @note: the intersection ray dir[3] is NOT normalized.
// Adopted from VTK's vtkBox intersection
bool
  LineToBoxIntersection (float box_bounds[6], float line_origin[3], float line_dir[3])
{
  const int _right = 0, _left = 1, _middle = 2;
  bool inside = true;   // start by assuming that the line origin is inside the box
  char    quadrant[3];
  int     whichPlane = 0;
  double  maxT[3], candidatePlane[3];

  //  First find closest planes
  for (int d = 0; d < 3; d++)
  {
    if (line_origin[d] < box_bounds[2*d])
    {
      quadrant[d] = _left;
      candidatePlane[d] = box_bounds[2*d];
      inside = false;
    }
    else if (line_origin[d] > box_bounds[2*d+1])
    {
      quadrant[d] = _right;
      candidatePlane[d] = box_bounds[2*d+1];
      inside = false;
    }
    else
    {
      quadrant[d] = _middle;
    }
  }

  //  Check whether origin of ray is inside bbox
  if (inside)
    return (true);

  //  Calculate parametric distances to plane
  for (int d = 0; d < 3; d++)
  {
    if (quadrant[d] != _middle && line_dir[d] != 0.0)
      maxT[d] = (candidatePlane[d] - line_origin[d]) / line_dir[d];
    else
      maxT[d] = -1.0;
  }

  //  Find the largest parametric value of intersection
  for (int d = 0; d < 3; d++)
    if (maxT[whichPlane] < maxT[d])
      whichPlane = d;

  //  Check for valid intersection along line
  if (maxT[whichPlane] > 1.0 || maxT[whichPlane] < 0.0)
    return (false);

  //  Intersection point along line is okay.  Check bbox.
  for (int d = 0; d < 3; d++)
  {
    if (whichPlane != d)
    {
      double coord = line_origin[d] + maxT[whichPlane]*line_dir[d];
      if (coord < box_bounds[2*d] || coord > box_bounds[2*d+1])
        return (false);
    }
  }
  return (true);
}

/* ---[ */
int
  main (int argc, char** argv)
{
#if GUI
  vtkRenderer *ren = vtkRenderer::New ();
  ren->SetBackground (1, 1, 1);
  vtkRenderWindowInteractor* iren = CreateRenderWindowAndInteractor (ren, "Voxelized Feature Estimation", argc, argv);
#endif
  
  double width = LEAF_WIDTH;
  bool interactive = false;

  if (argc < 2)
  {
    print_error (stderr, "Syntax is: VoxelizedFeatureEstimation <input>.pcd <output>.gfpfh <options>\n");
    fprintf (stderr, "  if <output>.pcd is not given, "); print_value (stderr, "stdout"); fprintf (stderr, " will be used instead\n");
    fprintf (stderr, "  where options are:\n");
    fprintf (stderr, "                    -width X = specify the leaf width (default "); print_value (stderr, "%g", width); fprintf (stderr, ")\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                    -interactive 0/1 = interactive mode ? (default "); print_value (stderr, "%s", BOOL2STR_ED (interactive)); fprintf (stderr, ")\n");
    return (-1);
  }

  ParseArgument (argc, argv, "-interactive", interactive);
  print_info (stderr, "Interactive mode: "); print_value (stderr, "%s\n", BOOL2STR_ED (interactive));
  ParseArgument (argc, argv, "-width", width);
  print_info (stderr, "Using a leaf width of "); print_value (stderr, "%g\n", width);

  // Take only the first .pcd file into account
  std::vector<int> pPCDFileIndices = ParseFileExtensionArgument (argc, argv, ".pcd");
  if (pPCDFileIndices.size () < 1)
  {
    print_error (stderr, "No .PCD file given as input!\n");
    return (-1);
  }

  std::vector<int> pGFPFHFileIndices = ParseFileExtensionArgument (argc, argv, ".gfpfh");
  if (pPCDFileIndices.size () < 1)
  {
    print_error (stderr, "No .PCD file given as input!\n");
    return (-1);
  }

  // Load the points from file
  PCD_Header header;
  print_info (stderr, "Loading "); print_value (stderr, "%s... ", argv[pPCDFileIndices.at (0)]);
  ANNpointArray points = LoadPCDFile (argv[pPCDFileIndices.at (0)], header);
  if (points == NULL)
    return (-1);
  fprintf (stderr, "[done : "); print_value (stderr, "%d %d", header.nr_points, header.dimID.size ()); fprintf (stderr, "D points]\n");
  print_info (stderr, "Available dimensions: "); print_value (stderr, "%s\n", getAvailableDimensions (header).c_str ());

  int xIdx  = getIndex (header, "x");
  if (xIdx == -1)
  {
    print_error (stderr, "XYZ point information not available!\n");
    return (-1);
  }
  int reg_idx  = getIndex (header, "reg");
  if (reg_idx == -1)
  {
    print_error (stderr, "Region point information not available!\n");
    return (-1);
  }
  
  // TODO: not needed
  vector<vector<int> > regions = getRegionInformationFromPoints (points, header.nr_points, reg_idx, -1, false);

  /// Create a fixed-size octree decomposition
  ANNpoint minB = annAllocPt (3); for (int d = 0; d < 3; d++) minB[d] = header.minPD[d];
  ANNpoint maxB = annAllocPt (3); for (int d = 0; d < 3; d++) maxB[d] = header.maxPD[d];

  /// TODO: cloud_octree::Octree (float cx, float cy, float cz, float dx, float dy, float dz, int max_depth);
  LevelOctree<vector<int> > *tree = CreateOctreeFromANN (points, header, width, minB, maxB);

#if GUI
  vtkAppendPolyData *octree_cubes = vtkAppendPolyData::New ();
  GetOctreeCuboids (tree, octree_cubes);
  vtkActor* cubes = createActorFromDataSet (octree_cubes->GetOutput (), 0.0, 0.0, 0.0, false);
  cubes->GetProperty ()->SetRepresentationToWireframe ();
  cubes->GetProperty ()->SetLineWidth (3);
  cubes->GetProperty ()->SetOpacity (0.1);
  ren->AddActor (cubes);
  vtkActor *lineActor = NULL, *cenActor_i = NULL, *cenActor_j = NULL, *cubeActor_i = NULL, *cubeActor_j = NULL;
#endif

  set<LinearOctreeIndex> cells;
  // Select only the occupied leaves
  // TODO: getOccupiedLeaves
  getOccupiedCells (tree, points, header.nr_points, cells);
  print_info (stderr, "Creating an octree representation of level "); print_value (stderr, "%d", tree->n_level); fprintf (stderr, " with "); print_value (stderr, "%d", cells.size ());
  fprintf (stderr, " voxels ("); print_value (stderr, "%d x %d x %d", tree->size (), tree->size (), tree->size ()); fprintf (stderr, ")\n");

   // Iterate through the cells
  set<LinearOctreeIndex>::iterator it_i, it_j;
  int X_i[3], X_j[3];

  ANNpoint centroid_i = annAllocPt (3);
  ANNpoint centroid_j = annAllocPt (3);
  ANNpoint centroid_ij = annAllocPt (3);

  // Iterate over all cells
  float line_p1[3], line_p2[3], box_bounds[6];
  for (it_i = cells.begin (); it_i != cells.end (); ++it_i)
  {
    // Get a cell
    LinearOctreeIndex loi_i = *it_i;
    /// TODO leaf_it->cen[0] leaf_it->cen[1] leaf_it->cen[2]
    loi_i.get (X_i[0], X_i[1], X_i[2]);

    // Get its contents
    /// TODO leaf_it->getIndices ()
    if (((*tree) (X_i[0], X_i[1], X_i[2])).size () == 0)
      continue;

#if DEBUG_GUI
    getLeafBounds (tree, X_i[0], X_i[1], X_i[2], box_bounds);
    //6703 [0.8831300139, -0.1859399974, -0.1285900027]
    if (!isPointInBounds (points[6703], box_bounds))
      continue;
#endif

    //ANNpoint centroid_i = cANN::computeCentroid (points, 3, ((*tree) (X_i[0], X_i[1], X_i[2])));
    getLeafCenter (tree, X_i, centroid_i);
#if GUI
      if (cenActor_i != NULL) ren->RemoveActor (cenActor_i);
      cenActor_i = createActorFromDataSet (ANNToVTK (centroid_i), 0.0, 1.0, 0.0, 25.0, false);
      ren->AddActor (cenActor_i);

      if (cubeActor_i != NULL) ren->RemoveActor (cubeActor_i);
      cubeActor_i = createActorFromDataSet (GetOctreeCuboid (tree, X_i), 0.0, 1.0, 0.0, true);
      cubeActor_i->GetProperty ()->SetRepresentationToSurface ();
      cubeActor_i->GetProperty ()->SetOpacity (0.2);
      ren->AddActor (cubeActor_i);
#endif

#if DEBUG_GUI
    for (it_j = cells.begin (); it_j != cells.end (); ++it_j)
#else
    for (it_j = it_i /*cells.begin ()*/; it_j != cells.end (); ++it_j)
#endif
    {
     /// TODO: just start for from it_i+1
     if (it_i == it_j)
       continue;

      // Get a cell
      LinearOctreeIndex loi_j = *it_j;
      loi_j.get (X_j[0], X_j[1], X_j[2]);

      // Get its contents
      if (((*tree) (X_j[0], X_j[1], X_j[2])).size () == 0)
        continue;

#if DEBUG_GUI
    getLeafBounds (tree, X_j[0], X_j[1], X_j[2], box_bounds);
    //7726 [0.8625100255, -0.1387699991, -0.1470099986]
    //7099 [0.8617900014, -0.1457899958, -0.1385599971]
    if (!isPointInBounds (points[7099], box_bounds))
      continue;
#endif

      //ANNpoint centroid_j = cANN::computeCentroid (points, 3, ((*tree) (X_j[0], X_j[1], X_j[2])));
      getLeafCenter (tree, X_j, centroid_j);

      // Check the entire tree
      int minX = 0, maxX = tree->size ();
      int minY = 0, maxY = tree->size ();
      int minZ = 0, maxZ = tree->size ();

      minX = min (X_i[0], X_j[0]); maxX = max (X_i[0], X_j[0]);
      minY = min (X_i[1], X_j[1]); maxY = max (X_i[1], X_j[1]);
      minZ = min (X_i[2], X_j[2]); maxZ = max (X_i[2], X_j[2]);

#if GUI
      // Create a vtkLineSource between centroid_i and centroid_j
      if (lineActor != NULL) ren->RemoveActor (lineActor);

      if (cenActor_j != NULL) ren->RemoveActor (cenActor_j);
      cenActor_j = createActorFromDataSet (ANNToVTK (centroid_j), 0.0, 0.0, 1.0, 25.0, false);
      ren->AddActor (cenActor_j);

      if (cubeActor_j != NULL) ren->RemoveActor (cubeActor_j);
      cubeActor_j = createActorFromDataSet (GetOctreeCuboid (tree, X_j), 0.0, 0.0, 1.0, true);
      cubeActor_j->GetProperty ()->SetRepresentationToSurface ();
      cubeActor_j->GetProperty ()->SetOpacity (0.2);
      ren->AddActor (cubeActor_j);

      lineActor = createVTKLine (centroid_i[0], centroid_i[1], centroid_i[2], centroid_j[0], centroid_j[1], centroid_j[2], 0.0002);
      lineActor->GetProperty ()->SetColor (1, 0, 0);
      ren->AddActor (lineActor);
      if (interactive)
        iren->Start ();
#endif

      /// TODO: line defined by a point and direction: p = 1st centroid, dir = 2nd - 1st centroid
      for (int d = 0; d < 3; d++)
      {
        line_p1[d] = centroid_i[d];
        line_p2[d] = centroid_j[d];
        line_p2[d] -= line_p1[d];                       // get the line direction
      }

      int z, y, x;

      // Create a paired histogram vector which holds: a) the actual centroid value of the intersected voxel, b) the distance from start_voxel to voxel_i
      vector<pair<double, LeafStruc> > histogram_values;

      vtkActorCollection* intersected_leaves_actors = vtkActorCollection::New ();
      // Iterate over leaves
      for (z = minZ; z <= maxZ+1; ++z)
      {
        for (y = minY; y < maxY+1; ++y)
        {
          for (x = minX; x < maxX+1; ++x)
          {
            // Ignore first/last voxel
            if (x == X_i[0] && y == X_i[1] && z == X_i[2])
              continue;
            if (x == X_j[0] && y == X_j[1] && z == X_j[2])
              continue;

            // Get the bounds of this leaf
            // TODO computeCellBounds
            getLeafBounds (tree, x, y, z, box_bounds);

            // Check if we intersect with it
            if (LineToBoxIntersection (box_bounds, line_p1, line_p2))
            {
              // Get the centroid of this leaf
              getLeafCenter (tree, x, y, z, centroid_ij);

              // Compute the distance to the start leaf
              pair<double, LeafStruc> histogram_pair;
              histogram_pair.second.distance = _sqr_dist (centroid_ij, centroid_i);
              histogram_pair.second.x = x;
              histogram_pair.second.y = y;
              histogram_pair.second.z = z;
              histogram_pair.second.nr_points = ((*tree) (x, y, z)).size ();

              // Get its contents, if empty set to EMPTY_VALUE
              if (((*tree) (x, y, z)).size () == 0)
              {
                histogram_pair.first = EMPTY_VALUE;
              }
              else
              {
                // Compute the dominant point label in this voxel
//                histogram_pair.first = cANN::computeCentroid1D (points, ((*tree) (x, y, z)), reg_idx);
                histogram_pair.first = getDominantElement (points, &((*tree) (x, y, z)), reg_idx, &regions);
              }

              histogram_values.push_back (histogram_pair);
            }
            else
            {
              if (interactive)
                print_error (stderr, "No intersection at x,y,z = %d,%d,%d\n", x, y, z);
            }
          }
        }
      }

      // Sort the histogram according to the distance of the leaves to the start leaf
      sort (histogram_values.begin (), histogram_values.end (), histogramElementCompare);
      // Display the first voxel
//      std::cout << cANN::computeCentroid1D (points, ((*tree) (X_i[0], X_i[1], X_i[2])), reg_idx) << " ";
      std::cout << getDominantElement (points, &((*tree) (X_i[0], X_i[1], X_i[2])), reg_idx, &regions) << " ";
      for (unsigned int hi = 0; hi < histogram_values.size (); hi++)
      {
        std::cout << histogram_values[hi].first << " ";
#if GUI
        if (histogram_values[hi].second.nr_points > 0)
        {
          int x = histogram_values[hi].second.x, y = histogram_values[hi].second.y, z = histogram_values[hi].second.z;
          vtkActor *intersected_leaf_actor = createActorFromDataSet (GetOctreeCuboid (tree, x, y, z), 1.0, 0.0, 0.0, false);
          intersected_leaf_actor->GetProperty ()->SetOpacity (0.2);
          intersected_leaves_actors->AddItem (intersected_leaf_actor);

          ren->AddActor (intersected_leaf_actor);
          if (interactive)
            iren->Start ();
        }
#endif
      }
      // Display the last voxel
//      std::cout << cANN::computeCentroid1D (points, ((*tree) (X_j[0], X_j[1], X_j[2])), reg_idx) << std::endl;
      std::cout << getDominantElement (points, &((*tree) (X_j[0], X_j[1], X_j[2])), reg_idx, &regions) << std::endl;

#if GUI
      if (interactive)
      {
        print_warning (stderr, "All done. Preparing to clean...\n");
        iren->Start ();
      }
      intersected_leaves_actors->InitTraversal ();
      for (int ca = 0; ca < intersected_leaves_actors->GetNumberOfItems (); ca++)
      {
        vtkActor *cur_ila = intersected_leaves_actors->GetNextActor ();
        cur_ila->ReleaseGraphicsResources (iren->GetRenderWindow ());
        ren->RemoveActor (cur_ila);
      }
#endif
    }

  }

//  ren->AddActor (createActorFromDataSet (ANNToVTK (points, header, 0), 0.1, false));
#if GUI
  iren->Render (); iren->Start ();
#endif

  annDeallocPt (centroid_i); annDeallocPt (centroid_j); annDeallocPt (centroid_ij);
}
/* ]--- */
