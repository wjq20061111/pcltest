#include "feature.h"

typedef std::pair<std::string, std::vector<float> > vfh_model;

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
	int vfh_idx;
  // Load the file as a PCD
	try
	{
		pcl::PCLPointCloud2 cloud;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type; unsigned int idx;
		r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

		vfh_idx = pcl::getFieldIndex (cloud, "vfh");
		if (vfh_idx == -1)
			return (false);
		if ((int)cloud.width * cloud.height != 1)
			return (false);
	}
	catch (const pcl::InvalidConversionException&)
	{
		return (false);
	}

  // Treat the VFH signature as a single Point Cloud
	pcl::PointCloud <pcl::VFHSignature308> point;
	pcl::io::loadPCDFile (path.string (), point);
	vfh.second.resize (308);

	std::vector <pcl::PCLPointField> fields;
	getFieldIndex (point, "vfh", fields);

	for (size_t i = 0; i < fields[vfh_idx].count; ++i)
	{
		vfh.second[i] = point.points[0].histogram[i];
	}
	vfh.first = path.string ();
	return (true);
}


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
	int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
	delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
	ifstream fs;
	fs.open (filename.c_str ());
	if (!fs.is_open () || fs.fail ())
		return (false);

	std::string line;
	while (!fs.eof ())
	{
		getline (fs, line);
		if (line.empty ())
			continue;
		vfh_model m;
		m.first = line;
		models.push_back (m);
	}
	fs.close ();
	return (true);
}

// int
// main (int argc, char** argv)
// {

int nKSearch(pcl::PointCloud<pcl::VFHSignature308>::Ptr &targetvfhs,
	std::vector<std::pair<int,float> > &kdistance
	)
{
	int k = 6;
static int turn=0;
	double thresh = DBL_MAX;     // No threshold, disabled by default

  // if (argc < 2)
  // {
  //   pcl::console::print_error 
  //     ("Need at least three parameters! Syntax is: %s <query_vfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
  //   pcl::console::print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: "); 
  //   pcl::console::print_value ("%d", k); pcl::console::print_info (")\n");
  //   pcl::console::print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: "); 
  //   pcl::console::print_value ("%f", thresh); pcl::console::print_info (")\n\n");
  //   return (-1);
  // }

	std::string extension (".pcd");
	transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  // Load the test histogram
  //std::vector<int> pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	pcl::PCDWriter writer;
	writer.write<pcl::VFHSignature308>("temp_vfh.pcd",*targetvfhs,false);

	vfh_model histogram;
	if (!loadHist ("temp_vfh.pcd", histogram))
	{
		//pcl::console::print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]);
		return (-1);
	}

  //pcl::console::parse_argument (argc, argv, "-thresh", thresh);
  // Search for the k closest matches
  //pcl::console::parse_argument (argc, argv, "-k", k);
	thresh=50;
	k=16;
  //pcl::console::print_highlight ("Using "); pcl::console::print_value ("%d", k); pcl::console::print_info (" nearest neighbors.\n");

	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";

	std::vector<vfh_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;
  // Check if the data has already been saved to disk
	if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
			training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
		return (-1);
	}
	else
	{
		loadFileList (models, training_data_list_file_name);
		flann::load_from_file (data, training_data_h5_file_name, "training_data");
//		pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
//			(int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}

  // Check if the tree index has already been saved to disk
	if (!boost::filesystem::exists (kdtree_idx_file_name))
	{
		pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
		return (-1);
	}
	else
	{
		flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
		index.buildIndex ();
		nearestKSearch (index, histogram, k, k_indices, k_distances);
	}

  // Output the results on screen
//  pcl::console::print_highlight ("The closest %d neighbors for target are:\n", k);
	for (int i = 0; i < 3; ++i)
		{
		//	pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
		//		i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
			std::string filename=models.at (k_indices[0][i]).first.c_str ();
			
			if(filename.find("/cup/")!=std::string::npos)
			{
				std::pair<int,float> kdis(turn,k_distances[0][i]);
				//kdis = std::make_pair (turn,(int)k_distances[0][i]);
				kdistance.push_back(kdis);
			}
			else
			{
				std::cout<<filename<<std::endl;
				std::pair<int,float> kdis(turn,1000);
				kdistance.push_back(kdis);
			}
		}
		turn++;
}