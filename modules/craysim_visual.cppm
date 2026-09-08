module;

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <expected>

#include <MulticamScene.h>
#include <libEyeRenderer.h> // getCurrentEyeSamplesPerOmmatidium

// scene exists at global scope in libEyeRenderer.so
extern MulticamScene* scene;

export module craysim.visual;

import sm.mathconst;
import sm.vvec;
import sm.quaternion;
import sm.mat;
import sm.hdfdata;
import sm.algo;
import sm.geometry;
import sm.config;
import sm.random;

import mplot.tools;
import craysim.compoundray.interop; // mathplot <--> compoundray interoperability
import craysim.compoundray.ommatidium; // An Ommatidium structure built on sm::vec<>
import craysim.compoundray.eyevisual;
import mplot.instancedscattervisual;
import mplot.normalsvisual;
import mplot.coordarrows;

export import mplot.gl.version;
export import mplot.visual;
export import mplot.fps.profiler;
export import oces.reader;

import sm.random_walk;

// Reproduce controller functions for the mplot window for ease of use
export namespace craysim
{
    void print_help (const char* progname)
    {
        std::cout << "USAGE:\n" << progname << " -f <path to gltf scene>\n\n"
                  << "\t-h\tDisplay this help information.\n"
                  << "\t-f\tPath to a gltf scene file (absolute or relative to current "
                  << "working directory, e.g. './data/axis_coloured_blocks.gltf').\n";
    }

    // Flags class
    enum class options : std::uint32_t
    {
        blender_axes,     // Set true to transform glTF into Blender's z-up axes
        max_fps,          // If true, poll, instead of fps
        path_from_csv,    // Move the agent from a pre-defined sequence of 2D coordinates that give it a path
        csv_in_plane,     // If true, then csv playback is 2-dimensional. User has to provide the 'altitude' for the agent
        save_csv_positions, // Write out the actual 3D positions that the CSV found on the landscape out to a file
        api_movement,     // Client code sets a vec/quat or mat for movement in the next render_and_poll()
        homing_mode,      // A flag for a 'go home' mode. It's up to client code to decide what to do with this.
        have_film_director, // user passed a json config file for film direction
        making_movie,     // If true, we're making a movie
        no_follow_agent,  // If true DON'T follow the agent (for some movies, this is useful)
        breadcrumbs_csv,   // If true, show breadcrumbs for csv-specified movements
        breadcrumbs_keymv, // If true, show breadcrumbs for key-commanded movements
        breadcrumbs_api,   // If true, show breadcrumbs for API-commanded movements
        breadcrumbs_walk,  // If true, show breadcrumbs for random-walk movements
        find_collisions,  // If true, then find distances to objects that we might collide with. A kind of simulated lidar.
        visualize_collisions,  // If true, then show distances to objects that we might collide with.
        save_hdf5,        // If true, then save any output data in HDF5 (active in 'path_from_csv' mode)
        debug_mv,         // Open a debug h5 file (craysim.h5) and run compute_mesh_movement once for debug of NavMesh
        show_fps,         // If true, show the FPS in the fps_label
        show_movenum,     // If true, show the current movement counter in the fps_label
        move_by_flying,   // If false, then hug the landscape (whether movement is by key, api or whatever); if true, fly
        eye_is_hex,       // If true, the glTF encoded a file with .heye suffix (instead of .eye) indicating it is hexagonally arranged.
        can_exit          // If set, program can exit now
    };

    // craysim::parse_inputs returns this struct
    struct parsed_inputs
    {
        sm::flags<craysim::options> opts;
        std::string gltf_path = {};
        std::string json_config_path = {};
        std::string csv_path = {};
        std::string h5_path = {};
        std::string hovh = {};
        std::int32_t w = -1; // user-requested width
        std::int32_t h = -1;
        float agent_coord_len = 1.0f;
        bool make_movie = false;
    };

    // Parse cmd line to find the path and set options. Return filepath of main scene gltf file and any csv path
    parsed_inputs parse_inputs (std::int32_t argc, char* argv[])
    {
        parsed_inputs rtn;

        for (std::int32_t i = 0; i < argc; i++) {
            std::string arg = std::string(argv[i]);
            if (arg == "-h") {
                craysim::print_help (argv[0]);
                rtn.opts |= craysim::options::can_exit;
            } else if (arg == "-f") {
                rtn.gltf_path = std::string(argv[++i]);
            } else if (arg == "-F") {
                rtn.opts |= craysim::options::no_follow_agent;
            } else if (arg == "-b") {
                rtn.opts |= craysim::options::blender_axes;
            } else if (arg == "-x") {
                rtn.opts |= craysim::options::max_fps;
            } else if (arg == "-c") {
                rtn.opts |= craysim::options::path_from_csv;
                i++;
                rtn.csv_path = std::string(argv[i]);
            } else if (arg == "-j") {
                rtn.opts |= craysim::options::have_film_director;
                i++;
                rtn.json_config_path = std::string(argv[i]);
            } else if (arg == "-5") {
                rtn.opts |= craysim::options::save_hdf5;
            } else if (arg == "-d") {
                // Get width and height
                std::string wxh_str = std::string(argv[++i]);
                std::cout << "cmd str: " << wxh_str << std::endl;
                std::vector<std::string> wxh_v = mplot::tools::stringToVector (wxh_str, "x");
                if (wxh_v.size() > 1) {
                    rtn.w = std::stoi (wxh_v[0]);
                    rtn.h = std::stoi (wxh_v[1]);
                }
                std::cout << "Got w=" << rtn.w << ", h=" << rtn.h << std::endl;
            } else if (arg == "-g") {
                rtn.opts |= craysim::options::debug_mv;
            } else if (arg == "-H") {
                rtn.hovh = std::string(argv[++i]);
            } else if (arg == "-L") {
                rtn.agent_coord_len = std::stof (argv[++i]);
            } else if (arg == "-m") {
                rtn.make_movie = true;
            }
        }
        if (rtn.gltf_path.empty()) {
            craysim::print_help (argv[0]);
            rtn.opts |= craysim::options::can_exit;
        }


        // If csv_path had commas in it, then just use the first one.
        std::vector<std::string> cpaths = mplot::tools::stringToVector (rtn.csv_path, ",");
        if (!cpaths.empty()) { rtn.h5_path = cpaths[0]; }
        mplot::tools::stripFileSuffix (rtn.h5_path);
        if (rtn.h5_path.empty()) { rtn.h5_path = "trail"; }
        rtn.h5_path += ".h5";

        return rtn;
    }

    // For a given samples per omm, return a sensible number of loops over which to average fps, so
    // that fps takes around 1 sec to stabilize.
    constexpr std::uint32_t best_n_samples (std::int32_t samples_per_omm)
    {
        std::uint32_t best_n = 0;
        switch (samples_per_omm) {
        case 1:
        case 2:
        {
            best_n = 1024; // about a seconds worth
            break;
        }
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        {
            best_n = 512;
            break;
        }
        case 128:
        case 256:
        {
            best_n = 256;
            break;
        }
        case 512:
        {
            best_n = 128;
            break;
        }
        case 1024:
        case 2048:
        {
            best_n = 64;
            break;
        }
        default:
        {
            best_n = 32;
        }
        }
        return best_n;
    }

    // Read a simple csv with 2D coordinates, using first two entries on each line
    bool read_csv (const std::string& path, sm::vvec<sm::vec<float, 2>>& positions)
    {
        std::ifstream f (path.c_str(), std::ios::in);
        if (f.is_open() == false) { return false; }
        std::string line;
        std::vector<std::string> tokens;
        while (std::getline (f, line)) {
            sm::vec<float, 2> twodpos;
            // Tokenize line into the coordinates
            twodpos.set_from_str (line, ",");
            positions.push_back (twodpos);
        }
        return true;
    }

    template <int glver>
    struct visual : public mplot::Visual<glver>
    {
        using mc = sm::mathconst<float>;

        // When the program starts, how many samples per ommatidium/element do you want?
        std::int32_t samples_per_omm_default = 64;

        visual (std::int32_t width, std::int32_t height, const std::string& title, craysim::parsed_inputs& prog_opts,
                const std::int32_t samples_default = 64, const float _agent_gamma = 1.0f)
            : mplot::Visual<glver> (width, height, title)
        {
            this->sim_opts = prog_opts.opts;
            this->sim_opts.set (craysim::options::making_movie, prog_opts.make_movie);

            // Boilerplate memory alloc for compound-ray and turn off verbose logging.
            multicamAlloc(); setVerbosity (false);

            this->lightingEffects (true);
            // Use a non-default zFar as we are likely to use large environments
            this->zFar = 2400;
            // Rotate about the nearest VisualModel
            this->rotateAboutNearest (true);
            // Rotate about a scene vertical axis? true for landscapes, false for cubes/objects (Ctrl-k changes I think, at runtime)
            this->rotateAboutVertical (true);
            // A blue sky background colour by default (client code can change this)
            this->bgcolour = { 0.298f, 0.412f,  0.576f };
            // State defaults
            //this->vstate |= state::show_camframe;
            if (this->sim_opts.test(craysim::options::blender_axes)) {
                this->switch_scene_vertical_axis(); // to uz up
                this->updateCoordLabels ("X", "Y", "Z(up)");
            } else {
                this->updateCoordLabels ("X", "Y(up)", "Z");
                // We start rotated into a drone view initial orientation for taking pictures of the world.
                // Into craysim::visual (with the blender_axes==true equivalent)...
                sm::quaternion<float> def_q (sm::vec<float>::ux(), mc::pi_over_2); // non-blender only
                this->setSceneRotation (def_q);
            }

            this->samples_per_omm_default = samples_default;

            // We follow the agent as it moves by default, but craysim has a 'don't follow' option
            if (this->sim_opts.test (craysim::options::no_follow_agent)) {
                this->options.set (mplot::visual_options::viewFollowsVMTranslations, false);
            } else {
                this->options.set (mplot::visual_options::viewFollowsVMTranslations, true);
            }

            this->load (prog_opts.gltf_path);
            // Use a FPS profiling with a text object on screen
            this->addLabel ("", {0.36f, 0.0f, -0.1f}, this->fps_label);
            this->setup_camera();
            this->setup_oces();
            this->agent_eyevisual_gamma = _agent_gamma;
            this->setup_eyevisual();
            this->setup_breadcrumbs (1000u); // default 1000 breadcrumbs
            this->setup_agent_coords (prog_opts.agent_coord_len);
            this->setup_compass_coords (prog_opts.agent_coord_len);

            // For json film direction, first try a path based on any csv path we have
            if (prog_opts.json_config_path.empty() && !prog_opts.csv_path.empty()) {
                // If csv_path had commas in it, then just use the first one.
                std::vector<std::string> cpaths = mplot::tools::stringToVector (prog_opts.csv_path, ",");
                // Construct json path from csv path and try that
                std::string candidate_json = "";
                if (!cpaths.empty()) { candidate_json = cpaths[0]; }
                mplot::tools::stripFileSuffix (candidate_json);
                candidate_json += ".json";
                std::cout << "Attempt to open csv's partner json: " << candidate_json << std::endl;
                this->setup_film_director (candidate_json);
                if (!cpaths.empty()) { this->first_csv = cpaths[0]; }

            } else {
                if (!prog_opts.json_config_path.empty()) {
                    this->setup_film_director (prog_opts.json_config_path);
                }
            }

            this->record.init (prog_opts.h5_path, std::ios::out | std::ios::trunc);

            // Default, craysim-friendly scene trans/rotation (your viewpoint)
            this->setSceneTrans (sm::vec<float,3>{ 0.01f, -3.7f, -99.0f });
            this->setSceneRotation (sm::quaternion<float>{ 0.93f, 0.16f, -0.32f, -0.056f });
        }

        ~visual()
        {
            stop(); // stop compound-ray from running
            multicamDealloc(); // De-allocate compound-ray memory
        }

        void load (const std::string& gltfpath)
        {
            // Load the file
            this->path = gltfpath;
            this->basepath = this->path;
            std::cout << "Loading glTF file \"" << this->path << "\"..." << std::endl;
            mplot::tools::stripUnixFile (this->basepath);
            std::cout << "glTF dir: " << this->basepath << std::endl;
            loadGlTFscene (this->path.c_str(), (this->sim_opts.test (craysim::options::blender_axes)
                                                ? craysim::compoundray::blender_transform() : sutil::Matrix4x4::identity()));
            // Get the visual models from the scene
            craysim::compoundray::scene_to_visualmodels<glver> (scene, this, false); // true for 'make_navmeshes'
        }

        void setup_camera()
        {
            // We get the eye data path from the glTF file
            std::int32_t ncam = static_cast<std::int32_t>(getCameraCount());
            std::int32_t my_compound_camera = -1;
            for (std::int32_t ci = 0; ci < ncam; ++ci) {
                gotoCamera (ci);
                this->efpaths[ci] = getEyeDataPath();
                if (!this->efpaths[ci].empty()) {
                    my_compound_camera = ci;
                    std::cout << "my_compound_camera = " << my_compound_camera
                              << " (" << this->efpaths[ci] << ")" << std::endl;
                    if (this->efpaths[ci].find (".heye") != std::string::npos) {
                        // Assume we are using a hexagonally arranged eye
                        this->sim_opts.set (craysim::options::eye_is_hex);
                    }
                }
            }

            // Now switch to each compound ray camera and set the samples per ommatidium/element
            if (my_compound_camera != -1) {
                gotoCamera (0);
                std::int32_t csamp = getCurrentEyeSamplesPerOmmatidium();
                std::cout << "Current eye samples per ommatidium for camera 0 is " << csamp << std::endl;
                if (csamp < 32000) { changeCurrentEyeSamplesPerOmmatidiumBy (samples_per_omm_default - csamp); }
                // Set samples for other compound eyes in the scene
                nextCamera();
                std::uint32_t _camidx = scene->getCameraIndex();
                while (_camidx != 0) {
                    csamp = getCurrentEyeSamplesPerOmmatidium();
                    std::cout << "Current eye samples per ommatidium for camera " << _camidx << " is " << csamp << std::endl;
                    if (csamp < 32000) { changeCurrentEyeSamplesPerOmmatidiumBy (samples_per_omm_default - csamp); }
                    nextCamera();
                    _camidx = scene->getCameraIndex();
                }
            }
        }

        void set_samples_per_ommatidium (const std::int32_t samples)
        {
            gotoCamera (0);
            std::uint32_t _camidx = scene->getCameraIndex();
            if (_camidx == 0) {
                std::cout << "Can't set samples per ommatidium; no camera yet\n";
            }
            while (_camidx != 0) {
                std::int32_t csamp = getCurrentEyeSamplesPerOmmatidium();
                std::cout << "Current eye samples per ommatidium for camera " << _camidx << " is " << csamp << std::endl;
                setCurrentEyeSamplesPerOmmatidium (samples);
                nextCamera();
                _camidx = scene->getCameraIndex();
            }
        }

        void setup_oces()
        {
            // Use oces_reader to read in our eye data, esp. for the head. One eye to be an OCES eye?
            for (auto efp : this->efpaths) {
                std::string oces_path = efp.second;
                mplot::tools::stripFileSuffix (oces_path);
                oces_path += ".gltf";
                // Now try to open oces_path
                std::cout << "Attempt to load OCES file " << oces_path << "\n";
                this->oces_reader[efp.first].read (oces_path);
                if (oces_reader[efp.first].read_success == false) {
                    std::cout << "No associated OCES file for a head with this one.\n";
                } else {
                    std::cout << "Success loading OCES file " << oces_path << "\n";
                    // Make the hex-equivalent eye
                    if (this->sim_opts.test(craysim::options::eye_is_hex) == true) {
                        std::cout << "Set up the hex equivalent of the OCES eye...\n";
                        this->oces_reader[efp.first].setup_hexeye();
                    }
                    // Read the head and make a VisualModel
                    constexpr float gam = 2.222222222222222f;
                    oces_reader[efp.first].get_eye()->head_mesh.single_colour = {std::pow (0.345f, gam), std::pow (0.122f, gam), std::pow (0.082f, gam)};
                    break;
                }
            }
        }

        // In-scene visualization of our compound-eye
        void setup_eyevisual()
        {
            // We get the initial camera localspace. This also serves to reset the camera pose. This
            // is set in the GLTF file and note that it may be a LEFT HANDED coordinate system! We
            // update this if we have a landscape, and it is updated to the first pose 'on the land'
            sm::mat<float, 4> ics = craysim::compoundray::getCameraSpace (scene);
            this->initial_camera_space.translate (ics.translation()); // Right handed

            // Create an EyeVisual 'eye' in our scene just for camera 0
            auto eyevm = std::make_unique<craysim::compoundray::EyeVisual<glver>> (sm::vec<>{},
                                                                                   &this->ommatidia_datas[0],
                                                                                   this->get_ommatidia_ptr(0));
            eyevm->set_parent (this->get_id());
            eyevm->setViewMatrix (this->initial_camera_space);
            eyevm->name = "EyeVisual";
            eyevm->setGamma (this->agent_eyevisual_gamma);
            if (this->get_head_mesh(0) != nullptr) { eyevm->addMeshgroup (*this->get_head_mesh(0)); }
            eyevm->finalize();
            this->eyes[0] = this->addVisualModel (eyevm);
            // This eye is the followed VM. If you teleport somewhere (such as with csv_playback) you have to call this again.
            this->setFollowedVM (this->eyes[0]);
        }

        // Collision visualization, to debug collision detection
        void setup_collisvis (std::uint64_t max_cv)
        {
            if (this->cvisvp != nullptr) {
                // check max_bc same as max_instances
                if (max_cv == cvisvp->max_instances) {
                    // Nothing to do
                    return;
                }
                // Remove existing
                this->removeVisualModel (this->cvisvp);
                this->cvisvp = nullptr;
            }
            auto cvisv = std::make_unique<mplot::InstancedScatterVisual<glver>> (sm::vec<>{});
            cvisv->set_parent (this->get_id());
            cvisv->max_instances = max_cv;
            cvisv->radiusFixed = 0.004f;
            cvisv->marker_offset = cvisv->radiusFixed;
            cvisv->marker_offset_dirn = sm::vec<>::uy();
            cvisv->name = "collisvis";
            cvisv->finalize();
            this->cvisvp = this->addVisualModel (cvisv);
        }

        // Breadcrumb trail for max_bc breadcrumbs. Called at start of program, can be re-called
        void setup_breadcrumbs (std::uint64_t max_bc)
        {
            // Set default options
            this->sim_opts.set (craysim::options::breadcrumbs_csv, true);
            this->sim_opts.set (craysim::options::breadcrumbs_keymv, false);
            this->sim_opts.set (craysim::options::breadcrumbs_api, false);
            this->sim_opts.set (craysim::options::breadcrumbs_walk, true);

            if (this->isvp != nullptr) {
                // check max_bc same as max_instances
                if (max_bc == isvp->max_instances) {
                    // Nothing to do
                    return;
                }
                // Remove existing
                this->removeVisualModel (this->isvp);
                this->isvp = nullptr;
            }
            auto isv = std::make_unique<mplot::InstancedScatterVisual<glver>> (sm::vec<>{});
            isv->set_parent (this->get_id());
            isv->max_instances = max_bc;
            isv->radiusFixed = 0.004f;
            isv->marker_offset = isv->radiusFixed;
            isv->marker_offset_dirn = sm::vec<>::uy();
            isv->name = "breadcrumbs";
            isv->finalize();
            this->isvp = this->addVisualModel (isv);
        }

        void setup_agent_coords (const float len)
        {
            // Make CoordArrows axes to show our camera's localspace (and to help find our tiny ant)
            auto antca = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<>{});
            antca->set_parent (this->get_id());
            antca->em = 0.0f; // labels don't work so well
            antca->lengths = { len, len, len };
            antca->thickness = 0.6f;
            antca->finalize();
            this->agent_coords = this->addVisualModel (antca);
            this->agent_coords->name = "agent_coords";
            this->agent_coords->setViewMatrix (this->initial_camera_space);
        }

        void setup_compass_coords (const float len)
        {
            auto compass_coords_up = std::make_unique<mplot::CoordArrows<glver>> (sm::vec<float>{0.0f});
            compass_coords_up->set_parent (this->get_id());
            compass_coords_up->em = 0.0f;
            compass_coords_up->lengths = {len, len, len};
            compass_coords_up->lengths *= 1.2f;
            compass_coords_up->thickness = 0.3f;
            compass_coords_up->finalize();
            this->compass_coords = this->addVisualModel (compass_coords_up);
            this->compass_coords->name = "compass_coords";
            this->compass_coords->setViewMatrix (this->get_compass_matrix());
        }

        // Probably to go to mathplot
        std::string json_config_path = {};
        void setup_film_director (const std::string& path)
        {
            std::cout << __func__ << " called with path " << path << std::endl;
            std::string _path = path;
            try {
                this->directions.clear();
                if (path.empty() && !this->json_config_path.empty()) {
                    _path = this->json_config_path;
                }
                this->json_config_path = _path;

                this->film_director.init (_path);

                if (this->film_director.ready) {
                    // Get list of movement time points at which camera movements should be created
                    nlohmann::json directions = this->film_director.get ("directions");
                    // Iterate though directions
                    for (auto dirn : directions) {
                        sm::config c (dirn);

                        std::uint32_t t = c.get<std::uint32_t>("event_time", 0);
                        std::string et = c.get<std::string>("event_type", "unknown");

                        this->directions[t] = mplot::direction_data();
                        this->directions[t].id = t;
                        this->directions[t].transform_time_frames = c.get<std::uint32_t> ("transform_time_frames", 0u);
                        this->directions[t].min_jerk = c.get<bool> ("min_jerk", true);

                        if (et == "sceneview") {
                            this->directions[t].sceneview = c.get_vec<float, 16> ("sceneview");
                            this->directions[t].event = mplot::direction_event::sceneview;
                        } else if (et == "timed_translation") {
                            this->directions[t].translation = c.get_vec<float, 3> ("translation");
                            this->directions[t].event = mplot::direction_event::timed_translation;
                        } else if (et == "timed_rotation") {
                            this->directions[t].about_vert_angle = c.get<float> ("about_vert_angle_degrees", 0.0f) * sm::mathconst<float>::deg2rad;
                            this->directions[t].tilt_angle = c.get<float> ("tilt_angle_degrees", 0.0f) * sm::mathconst<float>::deg2rad;
                            this->directions[t].event = mplot::direction_event::timed_rotation;
                        } else if (et == "timed_transform") {
                            this->directions[t].sceneview = c.get_vec<float, 16> ("sceneview");
                            this->directions[t].event = mplot::direction_event::timed_transform;
                        } else if (et == "timed_orbit") {
                            this->directions[t].orbit_axis = c.get_vec<float, 3> ("orbit_axis");
                            this->directions[t].orbit_centre = c.get_vec<float, 3> ("orbit_centre");
                            this->directions[t].orbit_angle = c.get<float> ("orbit_angle", 360.0f) * sm::mathconst<float>::deg2rad;
                            this->directions[t].event = mplot::direction_event::timed_orbit;
                        } else {
                            std::cout << "Unknown event type\n";
                        }
                    }
                } else {
                    std::cout << "Failed to open JSON file '" << _path << "'\n";
                }

            } catch (const std::exception& e) {
                std::cout << "Failed to open JSON file '" << _path << "' (exception: " << e.what() << ")\n";
            }
        }

        void setup_random_walk (const std::uint32_t _n_steps = 1500, const std::uint32_t _a_tau = 150, const float _kappa = 100, const float _a_max = 100)
        {
            this->rrg = std::make_unique<sm::random_walk<float>>(_n_steps, _a_tau, _kappa, _a_max);
        }

        void clear_breadcrumbs()
        {
            this->move_counter = 0;
            this->target_move_counter = 0;
            this->last_breadcrumb_count = 0;
            this->breadcrumb_coords.clear();
            this->breadcrumb_data.clear();
            // Leave bc_clr/bc_alpha/bc_scale for now
            if (this->bc_clr.empty() || this->bc_alpha.empty() || this->bc_scale.empty()) {
                this->isvp->set_instance_data (this->breadcrumb_coords);
            } else {
                this->isvp->set_instance_data (this->breadcrumb_coords, this->bc_clr, this->bc_alpha, this->bc_scale);
            }
        }

        void add_breadcrumb (const sm::vec<>& bc_location)
        {
            if (this->isvp == nullptr) { return; }

            if (this->breadcrumb_coords.size() < this->isvp->max_instances) {
                this->breadcrumb_coords.push_back (bc_location);
                this->breadcrumb_data.push_back (0.0f); // dummy for now
            } else {
                this->breadcrumb_coords[this->move_counter % this->isvp->max_instances] = bc_location;
                // breadcrumb_data.push_back (0.0f); // dummy for now, to be flags.
            }
            if (this->bc_clr.empty() || this->bc_alpha.empty() || this->bc_scale.empty()) {
                this->isvp->set_instance_data (this->breadcrumb_coords);
            } else {
                this->isvp->set_instance_data (this->breadcrumb_coords, this->bc_clr, this->bc_alpha, this->bc_scale);
            }
        }

        void clear_collisvis()
        {
            if (this->cvisvp == nullptr) { this->setup_collisvis (this->n_collision_distances * 100); }
            this->cv_coords.clear();
            this->cvisvp->set_instance_data (this->cv_coords);
        }

        void add_collisvis (const sm::vec<>& _location)
        {
            if (this->cvisvp == nullptr) { this->setup_collisvis (this->n_collision_distances * 100); }
            if (this->cv_coords.size() < this->cvisvp->max_instances) {
                this->cv_coords.push_back (_location);
            } // else do nothing
            this->cvisvp->set_instance_data (this->cv_coords, this->cv_clr, this->cv_alpha, this->cv_scale);
        }

        // Get access to the landscape VisualModel by searching for a selection of model names
        //
        // \param search_names A comma-separated list of model names to search for. If multiple in
        // the list are present, match the first in the list.
        void find_landscape (const std::string& search_names)
        {
            constexpr bool debug_landscape = false;

            std::vector<std::string> names = mplot::tools::stringToVector (search_names, ",");

            mplot::VisualModel<glver>* vmp = nullptr;

            // for each name in l_names
            for (auto search_name : names) {

                if (land != nullptr) { break; } // This will correctly cause exit on a second call to this function

                this->init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
                while ((vmp = this->get_next_vm_accessor()) != nullptr) {
                    if (vmp->name == search_name) {
                        this->land = vmp;
                        this->land->make_navmesh (this->basepath);
                        if constexpr (debug_landscape) {
                            // Can add a NormalsVisual for debug
                            auto nrm = std::make_unique<mplot::NormalsVisual<glver>> (land);
                            nrm->set_parent (this->get_id());
                            nrm->scale_factor = 0.01f;
                            // Set options to show just the boundary edge
                            nrm->options.set (mplot::normalsvisual_flags::show_tri_normals, true);
                            nrm->options.set (mplot::normalsvisual_flags::show_gl_normals, false);
                            nrm->options.set (mplot::normalsvisual_flags::show_boundary_halfedges, true);
                            nrm->options.set (mplot::normalsvisual_flags::show_inner_halfedges, false); // Heavy lifting
                            nrm->options.set (mplot::normalsvisual_flags::show_boundary_next, false);
                            nrm->options.set (mplot::normalsvisual_flags::show_boundary_prev, false);
                            nrm->nextprev_offset = sm::vec<float>::uy() * 0.01f;
                            nrm->finalize();
                            this->addVisualModel (nrm);
                        }
                        break;
                    } // else model with that name does not match
                }
            }
        }

        void init_path_from_csv()
        {
            // Initial position comes from first entry in the csv
            std::cout << "Set initial position from csv\n";
            sm::vec<float> nextloc = { this->csv_positions[0][0], 0.0f, this->csv_positions[0][1] };
            nextloc -= sm::vec<>{ 0.5f, 0.0f, 0.5f }; // don't understand this on re-reading
            // Change camspace based on nextloc. nextloc in landscape coords, so cam_nextloc = landscape.location + nextloc;
            sm::vec<float> ltstr = this->land_to_scene.translation();
            sm::vec<float> cam_nextloc = nextloc;
            cam_nextloc[0] += ltstr[0];
            cam_nextloc[2] += ltstr[2]; // update only x and z
            sm::mat<float, 4> cnl;
            cnl.translate (cam_nextloc);
            this->set_camera_pose (cnl);
            this->move_counter = 1;
        }

        void hide_landscape (const std::string also_hide = "")
        {
            if (this->land != nullptr) { this->land->setHide (true); }
            this->init_vm_accessor(); // Using an accessor scheme to loop through all VMs in a scene
            if (!also_hide.empty()) {
                mplot::VisualModel<glver>* vmp = nullptr;
                while ((vmp = this->get_next_vm_accessor()) != nullptr) {
                    if (vmp->name == also_hide) {
                        vmp->setHide (true);
                    }
                }
            }
        }

        void setup_landscape()
        {
            if (this->land == nullptr) { return; } // should have called find_landscape() first

            std::cout << "Landscape name: " << this->land->name << " was found [" << (land->vpos_size() / 3) << " vertices]\n";
            this->land_to_scene = land->getViewMatrix();
            sm::mat<float, 4> camspace = craysim::compoundray::getCameraSpace (scene);

            if (this->sim_opts.test (craysim::options::path_from_csv) && !this->csv_positions.empty()) {
                this->init_path_from_csv();
            }

            // One way to find_triangle_hit is to use the bounding box centre of this->land, and
            // draw a vector from the camera position towards that centre. However, this can lead to
            // the hit point being confusingly far away from the place the camera was set in the
            // glTF.
            //
            // This would be the function call:
            // auto[hp_scene, _ti0] = this->land->navmesh->find_triangle_hit (camspace, this->land_to_scene, 100.0f);
            //
            // A more useful approach is to simply find the hit in the -scene_up direction:
            sm::vec<float> camloc_mf = (this->land_to_scene.inverse() * camspace * sm::vec<float>{}).less_one_dim();

            auto[hp_scene, _ti0] = this->land->navmesh->find_triangle_hit (this->land_to_scene, camloc_mf, this->scene_up * -100.0f);

            if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {
                // Set up our camera using the data obtained from find_triangle_hit()
                sm::mat<float, 4> cam_to_scene = this->land->navmesh->position_camera (hp_scene, this->land_to_scene, this->hoverheight);
                if (cam_to_scene != sm::mat<float, 4>::identity()) {
                    this->set_camera_pose (cam_to_scene);
                    std::cout << "Camera/agent pose in scene coordinates: " << (cam_to_scene * sm::vec<>{}) << std::endl;
                } else {
                    std::cout << "cam_to_scene is identity??\n";
                }
            } else {
                std::cout << "Failed to find the landscape; Camera position unchanged from glTF/CSV\n";
            }

            sm::mat<float, 4> _cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            std::cout << "Got camera pose matrix from scene:\n" << _cam_to_scene << std::endl;
            sm::vec<float> _lastloc = _cam_to_scene.translation();

            // Update initial_camera_space
            this->initial_camera_space.set_identity();
            this->initial_camera_space.translate (_lastloc);
        }

        // Reset the camera location
        void check_reset_camspace (sm::mat<float, 4>& cam_to_scene)
        {
            // reset to initial camera space if requested
            if (this->vstate.test (state::campose_reset_request) == true) {
                this->stop(); // cancel any active movements
                this->set_camera_pose (this->initial_camera_space);
                this->setup_film_director (std::string(""));

                this->clear_breadcrumbs();
                if (this->sim_opts.test (craysim::options::path_from_csv) && !this->csv_positions.empty()) {
                    this->init_path_from_csv();
                }

                sm::mat<float, 4> camspace = craysim::compoundray::getCameraSpace (scene);
                sm::vec<float> camloc_mf = (this->land_to_scene.inverse() * camspace * sm::vec<float>{}).less_one_dim();
                auto[hp_scene, _ti0] = this->land->navmesh->find_triangle_hit (this->land_to_scene, camloc_mf, this->scene_up * -100.0f);
                cam_to_scene = this->land->navmesh->position_camera (hp_scene, this->land_to_scene, this->hoverheight);
                this->set_camera_pose (cam_to_scene);
                this->vstate.reset (state::campose_reset_request);
                this->vstate.set (state::campose_was_reset); // client code can read (and reset) this flag
                // t-1 values:
                this->tm1_ti0 = _ti0;
                this->tm1_mv_camframe = {};
                this->tm1_cam_to_scene = cam_to_scene;
            }
        }

        // Detect changes in the compound-ray camera, and update all our EyeVisuals accordingly
        void detect_camera_changes()
        {
            std::uint32_t camidx = scene->getCameraIndex();
            std::uint32_t camidx_start = camidx;
            do {
                // Detect changes for compound ray camera camidx...
                if (this->last_eye_size.contains (camidx) == false) { this->last_eye_size[camidx] = 0u; }

                if (this->ommatidia_datas[camidx].size() == 0) {
                    if (isCompoundEyeActive()) { getCameraData (this->ommatidia_datas[camidx]); }
                }

                if (this->eyes.contains (camidx) == true) {
                    this->eyes[camidx]->ommatidia = this->get_ommatidia_ptr (camidx);

                    for (auto& oe : this->other_eyes[camidx]) { oe->ommatidia = this->get_ommatidia_ptr (camidx); }

                    // reinit or reinit colours
                    if (this->ommatidias.contains (camidx) && this->ommatidias[camidx] != nullptr) {
                        std::size_t curr_eye_size = this->ommatidias[camidx]->size();
                        if (curr_eye_size == 0 || curr_eye_size != this->last_eye_size[camidx]) {

                            this->eyes[camidx]->reinit();
                            if (camidx < this->other_eyes.size()) {
                                for (auto oe : this->other_eyes[camidx]) { oe->reinit(); }
                            }
                            this->last_eye_size[camidx] = curr_eye_size;

                        } else {

                            this->eyes[camidx]->reinitColours();
                            if (camidx < this->other_eyes.size()) {
                                for (auto oe : this->other_eyes[camidx]) { oe->reinitColours(); } // 4x faster to just reinitColours
                            }
                        }
                    }
                }
                nextCamera();
                camidx = scene->getCameraIndex();
            } while (camidx != camidx_start);
        }

        // Has the camera rotated since the last ime step? Returns true if rotation in any plane is greater than the threshold.
        bool camera_has_rotated (float rotation_threshold_rad = 0.01745f) // default threshold of ~1 degree
        {
            sm::mat<float, 4> curr_cam_to_scene = craysim::compoundray::getCameraSpace(scene);

            // If this is the first call, just store and return false
            if (tm1_cam_to_scene[0] == std::numeric_limits<float>::max()) {
                this->tm1_cam_to_scene = curr_cam_to_scene;
                return false;
            }

            // Extract the 3x3 rotation parts from both matrices
            sm::mat<float, 3, 3> curr_rot = curr_cam_to_scene.linear();
            sm::mat<float, 3, 3> prev_rot = this->tm1_cam_to_scene.linear();

            // Compute relative rotation: delta = prev^T * curr
            sm::mat<float, 3, 3> delta_rot = prev_rot.transpose() * curr_rot;

            // Extract Euler angles (pitch, roll, yaw) from the relative rotation matrix
            // Pitch (rotation around x-axis)
            float pitch = std::atan2 (delta_rot(2, 1), delta_rot(2, 2));

            // Roll (rotation around y-axis) - with clipping to avoid asin domain issues
            float roll_arg = std::clamp (-delta_rot(2, 0), -1.0f, 1.0f);
            float roll = std::asin (roll_arg);

            // Yaw (rotation around z-axis)
            float yaw = std::atan2 (delta_rot(1, 0), delta_rot(0, 0));

            // Find the largest rotation angle
            float max_rotation = std::max({std::abs(pitch), std::abs(roll), std::abs(yaw)});

            // Update stored frame for next call
            this->tm1_cam_to_scene = curr_cam_to_scene;

            return max_rotation > rotation_threshold_rad;
        }

        sm::mat<float, 4> get_compass_matrix ()
        {
            // Project camera forward (z-axis) onto the y=0 ground plane to get heading direction
            sm::mat<float,4> cam_to_scene = craysim::compoundray::getCameraSpace(scene);
            // Vector pointing in the camera's forward direction, defined as the z direction in the camera's frame
            sm::vec<float> cam_z = cam_to_scene.col(2).less_one_dim();
            cam_z.renormalize();
            // Project the camera downward onto the ground plane (normal to the ground plane is the "up" direction) to get the forward direction of the agent's heading.
            sm::vec<float> fwd = sm::geometry::vector_plane_projection (this->scene_up, cam_z);
            fwd.renormalize();

            auto compass_matrix = sm::mat<float, 4>::frombasis (this->scene_up.cross (fwd), this->scene_up, fwd);
            compass_matrix.pretranslate (cam_to_scene.translation());

            return compass_matrix;
        }

        // Obtain the current heading of the camera, with respect to the world/scene axes, where
        // Visual::scene_right is North and Visual::scene_out is East.
        // Return result in radians in range [0 2pi], with 0=N, pi/2=E, pi=S, 3pi/2=W.
        float get_compass_heading_rad() const
        {
            // What's the orientation of cam_to_scene wrt to scene? It's just the rotation of cam_to_scene.
            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            // Offset cam_to_scene back to origin
            cam_to_scene.pretranslate (-cam_to_scene.translation());
            // The camera's forwards direction is its z axis
            sm::vec<float> cam_fwds = (cam_to_scene * sm::vec<float>::uz()).less_one_dim();
            // Project this onto the scene ground plane
            sm::vec<float> cam_fwds_proj = sm::geometry::vector_plane_projection (this->scene_up, cam_fwds);
            // Now convert cam_fwds_proj to a 2D angle, using scene_right for N and scene_out for E.
            // Choose a 2D heading vector so that the vector's 2D angle matches the angle used on a
            // compass, which is 0 deg for N and +90 deg for E.
            sm::vec<float, 2> heading2d = {
                cam_fwds_proj.dot (this->scene_right), // N along 2D x axis
                cam_fwds_proj.dot (this->scene_out)    // E along 2D y axis
            };
            float heading_r = heading2d.angle();
            sm::algo::zero_to_twopi (heading_r);
            return heading_r;
        }

        // Returns the compass heading in degrees, as on a regular compass, with 0 degrees meaning N
        // and 90 degrees E.
        float get_compass_heading() const
        {
            return this->get_compass_heading_rad() * sm::mathconst<float>::rad2deg;
        }

        std::string compass_degrees_to_str (float deg) const
        {
            constexpr float sdha = 22.5f / 2.0f; // single direction half angle
            // The 'd' means divider
            constexpr float Nd = 0.0f + sdha;
            constexpr float NNEd = 22.5f + sdha;
            constexpr float NEd = 45.0f + sdha;
            constexpr float ENEd = 67.5f + sdha;
            constexpr float Ed = 90.0f + sdha;
            constexpr float ESEd = 112.5f + sdha;
            constexpr float SEd = 135.0f + sdha;
            constexpr float SSEd = 157.5f + sdha;
            constexpr float Sd = 180.0f + sdha;
            constexpr float SSWd = 202.5f + sdha;
            constexpr float SWd = 225.0f + sdha;
            constexpr float WSWd = 247.5f + sdha;
            constexpr float Wd = 270.0f + sdha;
            constexpr float WNWd = 292.5f + sdha;
            constexpr float NWd = 315.0f + sdha;
            constexpr float NNWd = 337.5f + sdha;

            std::string dir = "";
            if (deg > NNWd) { dir = "N"; }
            else if (deg > NWd) { dir = "NNW"; }
            else if (deg > WNWd) { dir = "NW"; }
            else if (deg > Wd) { dir = "WNW"; }
            else if (deg > WSWd) { dir = "W"; }
            else if (deg > SWd) { dir = "WSW"; }
            else if (deg > SSWd) { dir = "SW"; }
            else if (deg > Sd) { dir = "SSW"; }
            else if (deg > SSEd) { dir = "S"; }
            else if (deg > SEd) { dir = "SSE"; }
            else if (deg > ESEd) { dir = "SE"; }
            else if (deg > Ed) { dir = "ESE"; }
            else if (deg > ENEd) { dir = "E"; }
            else if (deg > NEd) { dir = "ENE"; }
            else if (deg > NNEd) { dir = "NE"; }
            else if (deg > Nd) { dir = "NNE"; }
            else { dir = "N"; }
            return dir;
        }

        // Return 'N' or 'NNE' or equivalent for the heading direction
        std::string get_compass_heading_str() const
        {
            return this->compass_degrees_to_str (this->get_compass_heading());
        }

        // A rotation only api
        void api_rotate()
        {
            rotateCamerasLocallyAround (this->api_cam_rotn_angle,
                                        this->api_cam_rotn_axis[0],
                                        this->api_cam_rotn_axis[1],
                                        this->api_cam_rotn_axis[2]);

            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        void api_move_over_land()
        {
            // Check vec/quat/matrix and then make mv_camframe
            rotateCamerasLocallyAround (this->api_cam_rotn_angle,
                                        this->api_cam_rotn_axis[0],
                                        this->api_cam_rotn_axis[1],
                                        this->api_cam_rotn_axis[2]);
            this->instantaneous_rotation = true;

            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            // move by this->api_cam_mv; along z (for now?)
            sm::vec<float> mv_camframe = this->api_cam_mv;
            sm::vec<float> lastloc = cam_to_scene.translation();
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            if (this->land == nullptr) { std::cerr << "api_move_flying: land is nullptr!\n"; }
            if (this->land->navmesh == nullptr) { std::cerr << "api_move_flying: land->navmesh is nullptr!\n"; }
            std::uint32_t ti0_sv = this->land->navmesh->ti0;
            try {
                cam_to_scene = this->land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, this->land_to_scene, this->hoverheight);
                // Now we have moved, can compute instantaneous velocity
                this->instantaneous_velocity = cam_to_scene.translation() - cam_to_scene_sv.translation();
                this->distance_moved += this->instantaneous_velocity.length();

                this->tm1_ti0 = ti0_sv;
                this->tm1_mv_camframe = mv_camframe;
                this->tm1_cam_to_scene = cam_to_scene_sv;

            } catch (const std::exception& e) {
                std::string msg (e.what());
                std::cout << "Exception: " << msg << std::endl;
                if (msg.find ("off-edge:") == 0) {
                    std::cout << "We went off the edge. API move not possible. Don't crash.\n";
                    this->land->navmesh->ti0 = ti0_sv;
                } else {
                    std::cout << "API-commanded move was not possible...\n";
                    {
                        // Duplicated code from key_move...
                        std::cout << "Saving compute_mesh_movement data\n";
                        std::cout << "mv_camframe: " << mv_camframe << " and tm1_mv_camframe: " << this->tm1_mv_camframe << std::endl;
                        std::cout << "cam_to_scene_sv is\n" << cam_to_scene_sv
                                  << "\nand tm1_cam_to_scene:\n" << this->tm1_cam_to_scene << std::endl;
                        sm::hdfdata dsv ("./craysim.h5", std::ios::out | std::ios::trunc);
                        dsv.add_contained_vals ("/mv_camframe", mv_camframe);
                        dsv.add_contained_vals ("/cam_to_scene", cam_to_scene_sv.arr);
                        dsv.add_contained_vals ("/land_to_scene", this->land_to_scene.arr);
                        dsv.add_val ("/hoverheight", this->hoverheight);
                        dsv.add_val ("/ti0", ti0_sv);
                        // Also save t-1 values:
                        dsv.add_contained_vals ("/tm1_mv_camframe", this->tm1_mv_camframe);
                        dsv.add_contained_vals ("/tm1_cam_to_scene", this->tm1_cam_to_scene.arr);
                        dsv.add_val ("/tm1_ti0", this->tm1_ti0);
                    }
                    throw e;
                }
            }

            this->set_camera_pose (cam_to_scene);

            if (this->sim_opts.test (craysim::options::breadcrumbs_api)) {
                ++this->move_counter;
                this->add_breadcrumb (lastloc);
            }

            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        void api_move_flying ()
        {
            // Check vec/quat/matrix and then make mv_camframe
            rotateCamerasLocallyAround (this->api_cam_rotn_angle,
                                        this->api_cam_rotn_axis[0],
                                        this->api_cam_rotn_axis[1],
                                        this->api_cam_rotn_axis[2]);
            this->instantaneous_rotation = true;

            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            // move by this->api_cam_mv; along z (for now?)
            sm::vec<float> mv_camframe = this->api_cam_mv;
            sm::vec<float> lastloc = cam_to_scene.translation();
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            if (this->land == nullptr) { std::cerr << "api_move_flying: land is nullptr!\n"; }
            if (this->land->navmesh == nullptr) { std::cerr << "api_move_flying: land->navmesh is nullptr!\n"; }

            // Simpler than api_move_over_land:
            cam_to_scene.translate (mv_camframe);

            this->set_camera_pose (cam_to_scene);

            if (this->sim_opts.test (craysim::options::breadcrumbs_api)) {
                ++this->move_counter;
                this->add_breadcrumb (lastloc);
            }

            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        void api_move()
        {
            if (this->sim_opts.test (craysim::options::move_by_flying)) {
                this->api_move_flying();
            } else {
                this->api_move_over_land();
            }
        }

        // Set camera pose for all cameras. First find the triangle situated below the pose specified by cam_to_scene
        void set_camera_pose_and_ti0 (const sm::mat<float, 4>& cam_to_scene)
        {
            if (this->land == nullptr) {
                std::cout << "Cannot set ti0; there's no land model\n";
                return;
            }
            if (this->land->navmesh == nullptr) {
                std::cout << "Cannot set ti0; land has no navmesh\n";
                return;
            }
            // Find triangle hits using the scene's 'up' direction.
            sm::vec<float> camloc_mf = (this->land_to_scene.inverse() * cam_to_scene).translation();
            sm::vec<float> vnrm = this->scene_up;
            vnrm *= 4.0f;
            auto[hp_scene, _ti0] = this->land->navmesh->find_triangle_hit (this->land_to_scene, camloc_mf + (vnrm / 2.0f), -2.0f * vnrm, this->last_ti);
            if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {
                this->set_camera_pose (cam_to_scene);
            } else {
                std::cout << "Not setting camera pose, as no triangle hit was found for the pose\n" << cam_to_scene << std::endl;
            }
        }

        // Set camera pose for all cameras. This expects that ti0 is correctly set for the hit point of cam_to_scene through the landscape.
        void set_camera_pose (const sm::mat<float, 4>& cam_to_scene)
        {
            std::uint32_t camidx = scene->getCameraIndex();
            std::uint32_t camidx_start = camidx;
            do {
                setCameraPoseMatrix (craysim::compoundray::mat44_to_Matrix4x4 (cam_to_scene));
                nextCamera();
                camidx = scene->getCameraIndex();
            } while (camidx != camidx_start);
        }

        void key_move (const float fps)
        {
            if (this->sim_opts.test (craysim::options::move_by_flying)) {
                this->key_move_flying (fps);
            } else {
                this->key_move_over_land (fps);
            }
        }

        void key_move_flying (const float fps)
        {
            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            if (this->is_actively_rotating()) {
                this->instantaneous_rotation = true;
                // Up-down (pitch) is rotation about local camera frame axis x
                rotateCamerasLocallyAround (this->get_vertical_rotation_angle(), 1.0f, 0.0f, 0.0f);
                // Left-and-right (yaw) is rotation about local camera frame axis y
                rotateCamerasLocallyAround (this->get_horizontal_rotation_angle(), 0.0f, 1.0f, 0.0f);
                // Roll
                rotateCamerasLocallyAround (this->get_roll_rotation_angle(), 0.0f, 0.0f, 1.0f);
                cam_to_scene = craysim::compoundray::getCameraSpace (scene); // update
            }
            if (this->is_actively_translating()) {
                // Simpler than key_move_over_landscape
                sm::vec<float> lastloc = cam_to_scene.translation();
                sm::vec<float> mv_camframe = this->get_movement_vector (fps);
                cam_to_scene.translate (mv_camframe);
                this->set_camera_pose (cam_to_scene);
                if (this->sim_opts.test (craysim::options::breadcrumbs_keymv)) {
                    // Add a breadcrumb at the previous location
                    ++this->move_counter;
                    this->add_breadcrumb (lastloc);
                }
            }
            this->check_reset_camspace (cam_to_scene); // if requested

            // Update the view matrix of eye and eye localspace axes
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        // Make a keyboard based movement over the landscape
        void key_move_over_land (const float fps)
        {
            if (isCompoundEyeActive()) { // FIXME: I don't think this stanza is necessary here.
                auto _camidx = scene->getCameraIndex();
                this->ommatidias[_camidx] = &scene->m_ommVecs[_camidx];
            }

            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            if (this->is_actively_rotating()) {
                this->instantaneous_rotation = true;
                // Up-down (pitch) is rotation about local camera frame axis x
                rotateCamerasLocallyAround (this->get_vertical_rotation_angle(), 1.0f, 0.0f, 0.0f);
                // Left-and-right (yaw) is rotation about local camera frame axis y
                rotateCamerasLocallyAround (this->get_horizontal_rotation_angle(), 0.0f, 1.0f, 0.0f);
                // Roll
                rotateCamerasLocallyAround (this->get_roll_rotation_angle(), 0.0f, 0.0f, 1.0f);
                cam_to_scene = craysim::compoundray::getCameraSpace (scene); // update
            }
            if (this->is_actively_translating()) {
                if (this->move_state.test (craysim::visual<glver>::move_sense::up)) {
                    this->hoverheight += 0.0001f;
                } else if (this->move_state.test (craysim::visual<glver>::move_sense::down)) {
                    this->hoverheight -= 0.0001f;
                    if (this->hoverheight < 0.0f) { this->hoverheight = 0.0f; }
                }
                sm::vec<float> mv_camframe = this->get_movement_vector (fps);
                sm::vec<float> lastloc = cam_to_scene.translation();
                sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
                std::uint32_t ti0_sv = 0u;
                try {
                    if (this->land == nullptr) {
                        throw std::runtime_error ("Cannot compute_mesh_movement as there is no land to move over");
                    }
                    if (this->land->navmesh == nullptr) {
                        throw std::runtime_error ("Cannot compute_mesh_movement as there is no navmesh");
                    }
                    ti0_sv = this->land->navmesh->ti0;
                    cam_to_scene = this->land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, this->land_to_scene, this->hoverheight);
                    // Now we have moved, can compute instantaneous velocity
                    this->instantaneous_velocity = cam_to_scene.translation() - cam_to_scene_sv.translation();
                    this->distance_moved += this->instantaneous_velocity.length();

                    this->tm1_ti0 = ti0_sv;
                    this->tm1_mv_camframe = mv_camframe;
                    this->tm1_cam_to_scene = cam_to_scene_sv;

                } catch (const std::exception& e) {
                    std::string msg (e.what());
                    std::cout << "Exception: " << msg << std::endl;
                    if (msg.find ("off-edge:") == 0) {
                        std::cout << "We went off the edge. Key move not possible. Don't crash.\n";
                        this->land->navmesh->ti0 = ti0_sv;
                    } else {
                        std::cout << "key-command move was not possible...\n";
                        {
                            std::cout << "Saving compute_mesh_movement data\n";
                            std::cout << "mv_camframe: " << mv_camframe << " and tm1_mv_camframe: " << this->tm1_mv_camframe << std::endl;
                            std::cout << "cam_to_scene_sv is\n" << cam_to_scene_sv
                                      << "\nand tm1_cam_to_scene:\n" << this->tm1_cam_to_scene << std::endl;
                            sm::hdfdata dsv ("./craysim.h5", std::ios::out | std::ios::trunc);
                            dsv.add_contained_vals ("/mv_camframe", mv_camframe);
                            dsv.add_contained_vals ("/cam_to_scene", cam_to_scene_sv.arr);
                            dsv.add_contained_vals ("/land_to_scene", this->land_to_scene.arr);
                            dsv.add_val ("/hoverheight", this->hoverheight);
                            dsv.add_val ("/ti0", ti0_sv);
                            // Also save t-1 values:
                            dsv.add_contained_vals ("/tm1_mv_camframe", this->tm1_mv_camframe);
                            dsv.add_contained_vals ("/tm1_cam_to_scene", this->tm1_cam_to_scene.arr);
                            dsv.add_val ("/tm1_ti0", this->tm1_ti0);
                        }
                        throw e;
                    }
                }

                // Move the camera to the new position
                this->set_camera_pose (cam_to_scene);

                if (this->sim_opts.test (craysim::options::breadcrumbs_keymv)) {
                    // Add a breadcrumb at the previous location
                    ++this->move_counter;
                    this->add_breadcrumb (lastloc);
                }
            }
            this->check_reset_camspace (cam_to_scene); // if requested

            // Update the view matrix of eye and eye localspace axes
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        void walk()
        {
            if (this->sim_opts.test (craysim::options::move_by_flying)) {
                this->walk_flying();
            } else {
                this->walk_over_land();
            }
        }

        // Really: perform a 2D random walk in a plane
        void walk_flying()
        {
            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            // A random walk mode
            if (!this->rrg || this->vstate.test (craysim::visual<glver>::state::walk) == false) { return; }

            // set rotation and step length according to the Stone paper
            this->rrg->step();
            // rrg.omega is the angular speed rrg.speed is the linear speed
            rotateCamerasLocallyAround (this->rrg->omega, 0.0f, 1.0f, 0.0f);
            this->instantaneous_rotation = true;
            cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            // ti0, mv_camframe, cam_to_scene to save.
            sm::vec<float> mv_camframe = { 0, 0, this->rrg->speed };
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            cam_to_scene.translate (mv_camframe);
            this->instantaneous_velocity = cam_to_scene.translation() - cam_to_scene_sv.translation();
            this->distance_moved += this->instantaneous_velocity.length();
            if (this->sim_opts.test (craysim::options::breadcrumbs_walk)) {
                ++this->move_counter;
                this->add_breadcrumb (cam_to_scene_sv.translation());
            }
            this->set_camera_pose (cam_to_scene);
            this->check_reset_camspace (cam_to_scene); // if requested
            // Update the view matrix of eye and eye localspace axes
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        void walk_over_land()
        {
            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            // A random walk mode
            if (!this->rrg || this->vstate.test (craysim::visual<glver>::state::walk) == false) { return; }

            // set rotation and step length according to the Stone paper
            this->rrg->step();
            // rrg.omega is the angular speed rrg.speed is the linear speed
            rotateCamerasLocallyAround (this->rrg->omega, 0.0f, 1.0f, 0.0f);
            this->instantaneous_rotation = true;
            cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            // ti0, mv_camframe, cam_to_scene to save.
            sm::vec<float> mv_camframe = { 0, 0, this->rrg->speed };
            sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;
            std::uint32_t ti0_sv = this->land->navmesh->ti0;
            try {
                cam_to_scene = this->land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, this->land_to_scene, this->hoverheight);
                // Now we have moved, can compute instantaneous velocity
                this->instantaneous_velocity = cam_to_scene.translation() - cam_to_scene_sv.translation();
                this->distance_moved += this->instantaneous_velocity.length();

                this->tm1_ti0 = ti0_sv;
                this->tm1_mv_camframe = mv_camframe;
                this->tm1_cam_to_scene = cam_to_scene_sv;
                if (this->sim_opts.test (craysim::options::breadcrumbs_walk)) {
                    ++this->move_counter;
                    this->add_breadcrumb (cam_to_scene_sv.translation());
                }

            } catch (const std::exception& e) {
                std::string msg (e.what());
                std::cout << "Exception: " << msg << std::endl;
                if (msg.find ("off-edge:") == 0) {
                    std::cout << "We went off the edge. Change direction (rrg->about_turn()).\n";
                    this->rrg->about_turn();
                    this->land->navmesh->ti0 = ti0_sv;
                } else {
                    this->sim_opts.set (craysim::options::max_fps, false); // don't burn electricity after exception
                    this->vstate.set (craysim::visual<glver>::state::walk, false);
                    {
                        std::cout << "Saving compute_mesh_movement data\n"
                                  << "mv_camframe: " << mv_camframe << " and tm1_mv_camframe: " << this->tm1_mv_camframe
                                  << "\ncam_to_scene_sv is\n" << cam_to_scene_sv
                                  << "\nand tm1_cam_to_scene:\n" << this->tm1_cam_to_scene << std::endl;
                        sm::hdfdata dsv ("./craysim.h5", std::ios::out | std::ios::trunc);
                        dsv.add_contained_vals ("/mv_camframe", mv_camframe);
                        dsv.add_contained_vals ("/cam_to_scene", cam_to_scene_sv.arr);
                        dsv.add_contained_vals ("/land_to_scene", this->land_to_scene.arr);
                        dsv.add_val ("/hoverheight", this->hoverheight);
                        dsv.add_val ("/ti0", ti0_sv);
                        // Also save t-1 values:
                        dsv.add_contained_vals ("/tm1_mv_camframe", this->tm1_mv_camframe);
                        dsv.add_contained_vals ("/tm1_cam_to_scene", this->tm1_cam_to_scene.arr);
                        dsv.add_val ("/tm1_ti0", this->tm1_ti0);
                    }
                    throw e;
                }
            }
            this->set_camera_pose (cam_to_scene);
            this->check_reset_camspace (cam_to_scene); // if requested
            // Update the view matrix of eye and eye localspace axes
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);
        }

        bool csv_playback_one (bool allow_add_breadcrumb = false)
        {
            bool rtn = true;

            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);

            if (this->csv_positions.size() > this->move_counter) {

                if (this->directions.contains (this->move_counter)) {
                    std::cout << "Call setCurrentDirectionEvent (this->directions[" << this->move_counter << "])\n";
                    this->setCurrentDirectionEvent (this->directions[this->move_counter]);
                }

                /*
                 * With a csv path, teleport between each location (and then estimate the heading of
                 * the ant, or use csv_dirns). CSV positions are relative to the landscape model.
                 */
                sm::vec<float> lastcamloc = cam_to_scene.translation();

                sm::vec<float> nextloc = { this->csv_positions[this->move_counter][0], 0, this->csv_positions[this->move_counter][1] };
                sm::vec<float> lastloc = { this->csv_positions[this->move_counter - 1][0], 0, this->csv_positions[this->move_counter - 1][1] };
                //std::cout << "Teleport a distance " << (lastloc - nextloc).length() << std::endl;

                sm::vec<float> fwds;
                if (!this->csv_dirns.empty()) {
                    // Prolly a bit hacky. Does not take account of this->scene_up. Assumes it's the y axis.
                    fwds = { this->csv_dirns[this->move_counter][0], 0.0f, this->csv_dirns[this->move_counter][1] };
                } else {
                    fwds = nextloc - lastloc;
                }

                if (fwds.length() > 0.0f) {
                    sm::vec<float> ltstr = this->land_to_scene.translation(); // always the same (for each call to csv_playback)
                    sm::vec<float> cam_nextloc = nextloc;
                    cam_nextloc[0] += ltstr[0];
                    cam_nextloc[2] += ltstr[2]; // update only x and z

                    if (this->sim_opts.test (craysim::options::csv_in_plane) == true) {
                        // set the altitude from the last altitude
                        cam_nextloc[1] = lastcamloc[1]; // Also a bit hacky; need to use scene_up.
                        // position camera does the reorientation magic
                        if (fwds.length() > 0.0f) {
                            cam_to_scene = this->land->navmesh->position_camera (cam_nextloc, this->land_to_scene, 0.0f, fwds);
                        } // else agent position has not changed from the last time.
                    }

                    sm::mat<float, 4> cnl;
                    cnl.translate (cam_nextloc);
                    this->set_camera_pose (cnl);
                    cam_to_scene = craysim::compoundray::getCameraSpace (scene);
                }

                if (this->sim_opts.test (craysim::options::csv_in_plane) == true) {
                    // No more to do
                } else {
                    // Find triangle hits using the scene's 'up' direction.
                    sm::vec<float> camloc_mf = (this->land_to_scene.inverse() * cam_to_scene).translation();
                    sm::vec<float> vnrm = this->scene_up;
                    vnrm *= 4.0f;
                    auto[hp_scene, _ti0] = this->land->navmesh->find_triangle_hit (this->land_to_scene, camloc_mf + (vnrm / 2.0f), -2.0f * vnrm, this->last_ti);
                    this->last_ti = _ti0;

                    if (_ti0 != std::numeric_limits<std::uint32_t>::max()) {

                        // Set up our camera using the data obtained from find_triangle_hit()
                        if (fwds.length() > 0.0f) {
                            cam_to_scene = this->land->navmesh->position_camera (hp_scene, this->land_to_scene, this->hoverheight, fwds);
                        } // else agent position has not changed from the last time.

                        if (this->sim_opts.test (craysim::options::save_csv_positions)) {
                            if (this->csv_found_positions.empty()) {
                                this->csv_found_positions.resize (this->csv_positions.size());
                            }
                            this->csv_found_positions[this->move_counter] = cam_to_scene.translation();
                        }

                        if (cam_to_scene != sm::mat<float, 4>::identity()) {
                            this->set_camera_pose (cam_to_scene);
                        } else { std::cout << "csv_playback: cam_to_scene is identity?!\n"; }
                        // else what to do if cam_to_scene is identity?

                    } else {
                        // Rather than throwing, could just move on to next in csv?
                        cam_to_scene = craysim::compoundray::getCameraSpace (scene);
                        std::cout << "Omit csv_positions[this->move_counter] = csv_positions[" << this->move_counter << "] = "
                                  << this->csv_positions[this->move_counter] << " (failed to find triangle hit)\n";
                    }
                }

                // Can this go here? Removes two instances above
                // Now we have moved, can compute instantaneous velocity
                this->instantaneous_velocity = cam_to_scene.translation() - lastcamloc;
                this->distance_moved += this->instantaneous_velocity.length();

                if (allow_add_breadcrumb && this->sim_opts.test (craysim::options::breadcrumbs_csv)) {
                    if ((this->move_counter + 1) % breadcrumb_every == 0u && this->move_counter > this->last_breadcrumb_count) {
                        this->add_breadcrumb (lastcamloc);
                        this->last_breadcrumb_count = this->move_counter;
                    }
                }

            } else { // no more movements
                std::cout << "csv_playback: no more movements\n";
                rtn = false;
            }

            this->check_reset_camspace (cam_to_scene); // if requested
            // Update the view matrix of eye and eye loc->alspace axes
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (cam_to_scene); }
            this->agent_coords->setViewMatrix (cam_to_scene);

            // Update eyes[0]
            if ((this->move_counter - 1) == 1) {
                std::cout << "Update initial vm last locn\n";
                this->setFollowedVM (this->eyes[0]);
            }

            return rtn;
        }

        bool csv_playback()
        {
            bool rtn = true;

            if (target_move_counter < this->move_counter) {
                // Go straight there, no breadcrumbs
                this->move_counter = target_move_counter;
                csv_playback_one (false);
                return rtn;
            }

            while (this->move_counter <= target_move_counter) {
                if (csv_playback_one (true) == false) {
                    rtn = false;
                    break;
                }
                this->move_counter++;
            }

            return rtn;
        };

        // Debug a previously saved crash movement
        void do_crashed_movement ()
        {
            std::cout << "Loading compute_mesh_movement data from crash file\n";
            sm::mat<float, 4> _cam_to_scene = {{}};
            sm::mat<float, 4> _land_to_scene = {{}};
            sm::vec<float> _mv_camframe = {};
            float _hoverheight = 0.0f;
            std::uint32_t _ti0 = 0u;

            sm::hdfdata dsv ("./craysim.h5", std::ios::in);
            dsv.read_contained_vals ("/mv_camframe", _mv_camframe);
            dsv.read_contained_vals ("/cam_to_scene", _cam_to_scene.arr);
            dsv.read_contained_vals ("/land_to_scene", _land_to_scene.arr);
            dsv.read_val ("/hoverheight", _hoverheight);
            dsv.read_val ("/ti0", _ti0);
            dsv.read_contained_vals ("/tm1_mv_camframe", this->tm1_mv_camframe);
            dsv.read_contained_vals ("/tm1_cam_to_scene", this->tm1_cam_to_scene.arr);
            dsv.read_val ("/tm1_ti0", this->tm1_ti0);

            this->set_camera_pose (this->tm1_cam_to_scene);
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (this->tm1_cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (this->tm1_cam_to_scene); }
            this->agent_coords->setViewMatrix (this->tm1_cam_to_scene);
            std::cout << "First compute_mesh_movement from saved data:\n";
            this->land->navmesh->ti0 = this->tm1_ti0;
            sm::mat<float, 4> _cam_to_scene_1 = this->land->navmesh->compute_mesh_movement (this->tm1_mv_camframe, this->tm1_cam_to_scene, _land_to_scene, _hoverheight);
            std::cout << "\ncompute_mesh_movement for time t-1 returned cam_to_scene:\n" << _cam_to_scene_1 << "\n";
            //if (_cam_to_scene_1 != _cam_to_scene) { Random walk may have rotated the camera, to further alter cam_to_scene }
            std::cout << "Running second compute_mesh_movement from saved data:\n";
            this->land->navmesh->ti0 = _ti0;
            _cam_to_scene = this->land->navmesh->compute_mesh_movement (_mv_camframe, _cam_to_scene, _land_to_scene, _hoverheight);
            std::cout << "compute_mesh_movement for time t returned!\n";
            // Set the new position for camera and ant models
            this->set_camera_pose (_cam_to_scene);
            for (auto& eye : this->eyes) { if (eye.second != nullptr) { eye.second->setViewMatrix (_cam_to_scene); } }
            if (this->agent_body != nullptr) { this->agent_body->setViewMatrix (_cam_to_scene); }
            this->agent_coords->setViewMatrix (_cam_to_scene);
        }

        void start_loop_timer()
        {
            this->fps_profiler.at_begin (craysim::best_n_samples (getCurrentEyeSamplesPerOmmatidium()));
        }

        void end_loop_timer() { this->fps_profiler.at_end(); }

        // Allows client code to set up other windows etc
        std::vector<mplot::Visual<glver>*> other_windows = {};
        std::vector<mplot::Visual<glver>*> slow_windows = {};
        // How many fast renders to wait until we re-render the slow windows?
        std::uint64_t slow_every = 10u;

        // Time const for frames.
        double frame_tau = 0.0167;

        sm::mat<float, 4> random_rotation()
        {
            auto r = sm::mat<float, 4>::identity();
            // Add optional jitter/orientation uncertainty here.
            if (this->rotation_uncertainty_degrees[i_roll] > 0.0f) {
                // Sample an additional roll to add to cam_to_scene.
                float extra_roll = this->rotn_rng.get() * sm::mathconst<float>::deg2rad * this->rotation_uncertainty_degrees[i_roll];
                r.rotate (sm::vec<float>::uz(), extra_roll);
            }
            return r;
        }

        enum class collision_error
        {
            land_is_nullptr,
            navmesh_is_nullptr,
            move_not_possible
        };

        std::uint32_t n_collision_distances = 18;
        float collision_distance_max = std::numeric_limits<float>::max();
        // A vvec of tuples containing the collision distance (float) and model ID (the index into
        // VisualOwnable::vm for the model, cast to std::int32_t) of the model we'd hit. If model
        // ID is -1 it's the edge of the landscape. If model ID is -2, it's the end of the search
        // distance (say we try to collide with anything up to 2 m away)
        sm::vvec<std::tuple<float, std::int32_t>> agent_collision_distances = {};

        mplot::VisualModel<glver>* best_agent_body()
        {
            mplot::VisualModel<glver>* _agent = nullptr;
            if (this->agent_body != nullptr) {
                // We have an actual agent body to use
                _agent = this->agent_body;
            } else {
                // Fall back to using first EyeVisual as the agent
                _agent = static_cast<mplot::VisualModel<glver>*>(eyes[0]);
            }
            return _agent;
        }

        // Compare Bounding boxes for each model and our agent model
        std::int32_t test_agent_bounding_box_intersections (const sm::mat<float, 4>& agent_body_viewmatrix)
        {
            // std::cout << __func__ << " called for agent viewmatrix\n" << agent_body_viewmatrix << std::endl;

            std::int32_t rtn = -4;
            mplot::VisualModel<glver>* _agent = this->best_agent_body();

            // Oriented bounding box around virtual agent
            sm::mat<float, 3, 4> my_obb =  _agent->get_viewmatrix_obb (agent_body_viewmatrix);

            this->init_vm_accessor();
            mplot::VisualModel<glver>* mdl = this->get_next_vm_accessor();
            while (mdl) {
                // The EyeVisual model(s) in this->eyes follow the agent's own compound-eye
                // camera and never get a real bounding box computed (it stays at its initial
                // FLT_MAX/-FLT_MAX sentinel), which otherwise registers as an instant collision
                // in every direction. They're agent-attached visualisation props, not obstacles.
                bool is_own_eye = false;
                for (auto& eye_kv : this->eyes) { if (eye_kv.second == mdl) { is_own_eye = true; break; } }

                if (mdl != this->land
                    && mdl->name != "vegetation_inner_alternative" // hack to work in Seville environment
                    && mdl != isvp // breadcrumbs
                    && mdl != cvisvp // collision visualization
                    && mdl != _agent
                    && mdl != this->agent_coords
                    && mdl != this->compass_coords
                    && !is_own_eye) {

                    // Get the model's oriented bounding box (the compute in here may be
                    // repeated many times and is acandidate for optimization)
                    sm::mat<float, 3, 4> obb = mdl->get_viewmatrix_obb();
                    // Do oriented bounding box collision detection
                    if (sm::geometry::obb_collision_detect (my_obb, obb)) {
                        auto model_id = static_cast<std::int32_t>(this->getVisualModelId (mdl));
                        return model_id;
                    }

                } //else {std::cout << "Skipping " << mdl->name << std::endl; }
                mdl = this->get_next_vm_accessor();
            }

            return rtn;
        }

        /*
         * In direction @dirn play forward a 'virtual agent movement' until we hit either the
         * bounding box of a non-landscape model, or the edge of the landscape. Return a tuple
         * containing the distance to the collision (and which model ID it was, cast to int32_t). If
         * model ID is -1 it was the edge of the landscape. If model ID is -2, it was the end of the
         * search distance up @up_to agent sizes/bodylengths
         *
         * Note experimentation with std::expected in return type.
         */
        std::expected <std::tuple<float, std::int32_t>, collision_error> compute_collision_distance (const float dirn, const std::uint32_t up_to)
        {
            //std::cout << __func__ << " called to find collision in (ego) direction " << dirn << std::endl;
            if (this->land == nullptr) { return std::unexpected (collision_error::land_is_nullptr); }
            if (this->land->navmesh == nullptr) { return std::unexpected (collision_error::navmesh_is_nullptr); }

            std::tuple<float, std::int32_t> rtn = {};

            // Get the current camera space (could happen once in compute_collision_distances)
            sm::mat<float, 4> cam_to_scene = craysim::compoundray::getCameraSpace (scene);
            // Compute translations (could happen once in compute_collision_distances)
            sm::vec<float> cam_tran = cam_to_scene.translation();
            sm::mat<float, 4> tr1;
            tr1.translate (cam_tran);
            sm::mat<float, 4> tr2;
            tr2.translate (-cam_tran);
            // In the camera's frame, y is up (could happen once in compute_collision_distances)
            sm::vec<float> cam_y = ((tr2 * cam_to_scene) * sm::vec<float>::uy()).less_one_dim();

            // Set the angle of the camera space (must occur in this function)
            sm::mat<float, 4> rotn (sm::quaternion<float>(cam_y, dirn));

            // Rotate the virtual agent camera to our chosen direction
            cam_to_scene = tr1 * rotn * tr2 * cam_to_scene;

            // By computing mesh movement, we may change the navmesh's ti0, so we will need to reset it at the end
            std::uint32_t ti0_sv = this->land->navmesh->ti0;

            bool collided = false;

            // Get characteristic movement distance from agent BB
            mplot::VisualModel<glver>* _agent = this->best_agent_body();
            float agent_sz = _agent->bb.span().mean();
            // Want to scale agent_sz by the scaling present in get_viewmatrix.
            sm::vec<float, 3> agent_scale = _agent->getViewMatrix().scaling_vec();
            agent_sz *= agent_scale[0]; // Assume uniform scaling

            // Spacing between successive sampled distances along a ray, in units of agent_sz. The
            // overall search distance (agent_sz * up_to) is scaled by the same factor, so a ray
            // still gets roughly @up_to samples, just spread @sample_spacing_mult times farther
            // apart (and the total search radius grows by the same factor).
            constexpr float sample_spacing_mult = 20.0f; // 10.0 * 1.3

            float search_distance = agent_sz;
            float up_to_dist = agent_sz * up_to * sample_spacing_mult;

            // Create a movement wrt our camera forwards direction z.
            sm::vec<float> mv_camframe = {0, 0, agent_sz * sample_spacing_mult};

            while (!collided && search_distance < up_to_dist) {

                // Save a copy of the camera space
                sm::mat<float, 4> cam_to_scene_sv = cam_to_scene;

                try {
                    cam_to_scene = this->land->navmesh->compute_mesh_movement (mv_camframe, cam_to_scene, this->land_to_scene, this->hoverheight);

                    // Now we have moved, update search_distance
                    search_distance += (cam_to_scene.translation() - cam_to_scene_sv.translation()).length();
                    // Having moved, does the linear distance between last location and this location cross a bounding box?
                    std::int32_t id = this->test_agent_bounding_box_intersections (cam_to_scene);
                    if (id > 0) {
                        // BB says collision occurred
                        rtn = { search_distance, id };
                        collided = true;
                    } else {
                        if (search_distance >= up_to_dist) { rtn = { search_distance, -2 }; }
                    }

                } catch (const std::exception& e) {
                    std::string msg (e.what());
                    if (msg.find ("off-edge:") == 0) {
                        rtn = { search_distance, -1 }; // Went off edge, that's the collision.
                        collided = true;
                    } else {
                        return std::unexpected (collision_error::move_not_possible);
                    }
                }

                if (this->sim_opts.test (craysim::options::visualize_collisions)) {
                    // Show locn of cam_to_scene for this search point:
                    if (!collided) {
                        this->add_collisvis (cam_to_scene.translation());
                    }
                }
            }

            this->land->navmesh->ti0 = ti0_sv; // Back to the location of the actual camera (which was never moved)

            return rtn;
        }

        // Formats a tuple containing a distance and a model ID/search end info
        std::string format_collision_distance (const std::tuple<float, std::int32_t>& cd)
        {
            auto[dist, modelid] = cd;
            std::string mid = {};
            if (modelid < -2) {
                mid = "unexpected";
            } else if (modelid == -2) {
                mid = "max-search";
            } else if (modelid == -1) {
                mid = "land-edge";
            } else {
                mid = std::format ("vm {}", modelid);
            }
            return std::format("{:.2f} ({})", dist, mid);
        }

        // Find the distance, angle (degrees) and model ID to the closest collision between your
        // agent and a model on your landscape. The values in agent_collision_distances should have
        // been previously computed with a call to compute_collision_distances()
        std::tuple<float, float, std::int32_t> get_closest_collision_distance()
        {
            // distance, angle, modelid
            std::tuple<float, float, std::int32_t> rtn = {};

            const float ainc = 360.0f / this->agent_collision_distances.size();
            float angle = 0.0f;
            float closest = std::numeric_limits<float>::max();
            for (std::uint32_t i = 0; i < this->agent_collision_distances.size(); ++i) {
                auto[dist, modelid] = this->agent_collision_distances[i];
                if (dist < closest) {
                    rtn = {dist, angle, modelid};
                    closest = dist;
                }
                angle += ainc;
            }
            return rtn;
        }

        // Find the distance, angle (degrees) and model ID to the closest collision between your
        // agent and a model on your landscape and return as a formatted string. The values in
        // agent_collision_distances should have been previously computed with a call to
        // compute_collision_distances()
        std::string get_closest_collision_distance_str()
        {
            auto[dist, angle, modelid] = this->get_closest_collision_distance();
            std::tuple<float, std::int32_t> cd = { dist, modelid };
            return std::format ("Bearing {} collides in {}", angle, this->format_collision_distance (cd));
        }

        // From agent_collision_distances, get the distance to collision for the angle degrees
        // (using the best data we have). The values in agent_collision_distances should have been
        // previously computed with a call to compute_collision_distances()
        std::tuple<float, std::int32_t> get_collision_distance (const float degrees)
        {
            // Find closest available angle to degrees
            float rads = sm::mathconst<float>::deg2rad * degrees;
            sm::algo::zero_to_twopi (rads); // constrain
            const float ainc = sm::mathconst<float>::two_pi / this->agent_collision_distances.size();
            const float fidx = rads / ainc;
            const float fidx_c = std::ceil (fidx);
            const float fidx_f = std::floor (fidx);
            std::uint32_t idx = 0;
            if (fidx_c - fidx < fidx - fidx_f) {
                idx = static_cast<std::uint32_t>(fidx_c);
            } else {
                idx = static_cast<std::uint32_t>(fidx_f);
            }
            if (idx >= this->agent_collision_distances.size()) {
                throw std::runtime_error ("get_collision_distance: determined a bad index?");
            }
            return this->agent_collision_distances[idx];
        }

        std::string get_collision_distance_str (const float degrees)
        {
            return this->format_collision_distance (this->get_collision_distance (degrees));
        }

        // In some number of directions around the circle (say 360 at 1 degree intervals or 72 at 5
        // degree intervals) play forward a 'virtual agent movement' until we hit either the
        // bounding box of a non-landscape model, or the edge of the landscape. Record the distance
        // to the collision (and which model it was)
        void compute_collision_distances()
        {
            if (this->agent_collision_distances.size() != this->n_collision_distances) {
                this->agent_collision_distances.resize (this->n_collision_distances, {});
            }

            std::uint32_t up_to = 10; // Have space for ~50, but depends on roll/pitch/yaw of camera

            if (this->sim_opts.test (craysim::options::visualize_collisions)) {
                this->clear_collisvis();
                // Check we have enough vizualization space
                if (up_to * this->n_collision_distances > this->cvisvp->max_instances) {
                    std::cout << "Not enough space, set up_to lower\n";
                }
            }

            for (std::uint32_t i = 0; i < this->n_collision_distances; ++i) {
                float dirn = (sm::mathconst<float>::two_pi * i) / n_collision_distances;
                auto res = this->compute_collision_distance (dirn, up_to);
                if (res) { this->agent_collision_distances[i] = res.value(); }
            }
        }

        // Call this from your main loop. Returns true if slow windows were processed
        bool render_and_poll ()
        {
            // The current camera may have changed, this subroutine deals with any changes in this->eye and other_eyes
            this->detect_camera_changes(); // reinits the eyes

            // Now render the mathplot window
            this->render();
            // Change label after render (it needs v's context, not any of the other windows)
            if (this->move_counter % this->fps_label_update_period == 0) { this->fps_label_update(); }

            // Save some electricity while developing - limit to 60 FPS. For max speed use this->poll() (-x)
            if (this->sim_opts.test (craysim::options::max_fps)) { this->poll(); } else { this->wait (this->frame_tau); }

            // Render the other windows
            if ((this->render_counter % this->slow_every) == 0u) {
                for (auto swin : this->slow_windows) { swin->render(); }
            }
            for (auto owin : this->other_windows) { owin->render(); }

            // Deal with any movements commanded by key press events (including reset)

            this->setContext(); // right now key move over land needs main window's context

            this->instantaneous_velocity = {}; // velocity computed per render cycle
            this->instantaneous_rotation = false;

            this->agent_coords->setHide (!this->vstate.test(craysim::visual<glver>::state::show_camframe));
            if (this->compass_coords != nullptr) {
                this->compass_coords->setHide (!this->vstate.test(craysim::visual<glver>::state::show_compass));
            }

            // Any of the following movement-creating functions will set the instantaneous velocity
            //
            // walk/csv playback/check keys for movement command
            if (this->vstate.test (craysim::visual<glver>::state::paused) == false) {

                if (this->vstate.test (craysim::visual<glver>::state::walk)) {
                    this->walk();
                } else if (this->sim_opts.test (craysim::options::path_from_csv) && this->csv_positions.size() > this->move_counter) {
                    // Construct path from csv file of 2D agent locations
                    if (this->csv_playback() == false && this->sim_opts.test (craysim::options::making_movie)) {
                        // In movie mode, finish as soon as the movie is made
                        this->signal_to_quit();
                    }
                    this->target_move_counter++;

                } else if (this->sim_opts.test (craysim::options::path_from_csv)
                           && this->csv_positions.size() <= this->move_counter
                           && this->sim_opts.test (craysim::options::making_movie)) {
                    std::cout << "Ran out of moves making movies, signal to quit\n";
                    this->signal_to_quit();

                } else if (this->sim_opts.any_of ({craysim::options::api_movement, craysim::options::homing_mode})) {
                    // React to movements commanded by vec/quaternion or transformation matrix
                    // (i.e. by client code).
                    this->api_move();
                } else {
                    this->key_move (this->fps_profiler.fps_mean);
                }
            } else if (this->vstate.test (craysim::visual<glver>::state::paused) == true
                       && this->sim_opts.any_of ({craysim::options::api_movement, craysim::options::homing_mode})) {
                this->api_rotate(); // BUT don't inc move counter! This enables rotating while paused
            } else if (this->vstate.test (craysim::visual<glver>::state::paused) == true
                       && this->sim_opts.test (craysim::options::path_from_csv)
                       && this->csv_positions.size() > this->move_counter) {
                this->csv_playback();
            }

            // Having moved, if we need to, we can re-compute the distance to any non-landscape objects that we might collide with.
            if (this->sim_opts.test (craysim::options::find_collisions)
                && (this->instantaneous_velocity.length() > 0.0f || this->instantaneous_rotation == true)) {
                this->compute_collision_distances();

                // Here's how to access the information computed in compute_collision_distances(). You can call these after render_and_poll()
                // std::cout << "Safe distance in ego forwards: " << this->get_collision_distance_str (0.0f) << std::endl;
                // std::cout << "Closest safe distance: " << this->get_closest_collision_distance_str() << std::endl;
            }

            auto cam_pre_rand = sm::mat<float, 4>::identity();
            // If requested, add uncertainty in the pitch/roll/yaw for the image here, in a way that
            // does not affect the agent's movement.
            if (this->rotation_uncertainty_degrees.sum() > 0.0f) {
                sm::mat<float, 4> rr = this->random_rotation();
                cam_pre_rand = craysim::compoundray::getCameraSpace (scene);
                this->set_camera_pose (cam_pre_rand * rr); // rotate camera by rr
            }

            std::uint32_t camidx = 0;
            // Call the compound-ray ray casting method to recompute the compound-eye view of the scene
            renderFrame();

            // If necessary, restore camera rotation
            if (this->rotation_uncertainty_degrees.sum() > 0.0f) { this->set_camera_pose (cam_pre_rand); }

            // Access data so that a brain model could be fed
            if (isCompoundEyeActive()) {
                camidx = scene->getCameraIndex();
                getCameraData (this->ommatidia_datas[camidx]);
                this->ommatidias[camidx] = &scene->m_ommVecs[camidx];

                // if csv mode, then save the data (camidx 0 only)
                if (camidx == 0 && this->sim_opts.all_of ({craysim::options::path_from_csv, craysim::options::save_hdf5})
                    && this->csv_positions.size() > this->move_counter) {
                    std::cout << "Saving frame...\n";
                    std::string ommframe = "/ommatidia_data/frame_" + std::to_string (this->move_counter);
                    try {
                        record.add_contained_vals (ommframe.c_str(), this->ommatidia_datas[camidx]);
                    } catch (const std::exception& e) {} // Probably didn't move this time.
                }
            }

            // Render any other compound eyes in the scene
            if (this->ommatidia_datas.size() > 1) {
                nextCamera();
                std::uint32_t _camidx = scene->getCameraIndex();
                while (_camidx != camidx) {
                    renderFrame();
                    if (isCompoundEyeActive()) {
                        getCameraData (this->ommatidia_datas[_camidx]);
                        this->ommatidias[_camidx] = &scene->m_ommVecs[_camidx];
                    }
                    nextCamera();
                    _camidx = scene->getCameraIndex();
                }
            }

            // Scale size of breadcrumbs based on distance
            float iscl = this->bc_mult * std::log (1.0f + this->bc_mult * this->get_d_to_rotation_centre());
            this->isvp->set_instance_scale (iscl);
            if (this->cvisvp != nullptr) {
                this->cvisvp->set_instance_scale (iscl * 1.2f);
            }

            if (this->compass_coords != nullptr) {
                this->compass_coords->setViewMatrix (this->get_compass_matrix());
            }

            return (this->render_counter % this->slow_every) == 0u;
        }

        // Save once-only data into the recording file (ommatidia data)
        void complete_recording()
        {
            if (this->sim_opts.all_of ({craysim::options::path_from_csv, craysim::options::save_hdf5})) {
                // convert std::vector<Ommatidium>* ommatidia into vvecs that can be h5 saved
                auto ommat = this->get_ommatidia_ptr(0);
                sm::vvec<sm::vec<float, 3>> o_pos;
                sm::vvec<sm::vec<float, 3>> o_dir;
                sm::vvec<float> o_aa;
                sm::vvec<float> o_fo;
                for (auto o : *ommat) {
                    o_pos.push_back (o.relativePosition);
                    o_dir.push_back (o.relativeDirection);
                    o_aa.push_back (o.acceptanceAngleRadians);
                    o_fo.push_back (o.focalPointOffset);
                }
                std::cout << "Pos\n";
                this->record.add_contained_vals ("/ommatidia/relativePosition", o_pos);
                std::cout << "Dir\n";
                this->record.add_contained_vals ("/ommatidia/relativeDirection", o_dir);
                std::cout << "AA\n";
                this->record.add_contained_vals ("/ommatidia/acceptanceAngleRadians", o_aa);
                std::cout << "FO\n";
                this->record.add_contained_vals ("/ommatidia/focalPointOffset", o_fo);
                std::cout << "Completed recording" << std::endl;
            }
            if (!this->csv_found_positions.empty()) {
                std::string fp_filename = this->first_csv + ".3d.csv";
                std::cout << "Write out found 3D positions to " << fp_filename << "\n";
                std::ofstream fout (fp_filename, std::ios::out | std::ios::trunc);
                if (fout.is_open()) {
                    for (auto p : this->csv_found_positions) {
                        fout << p.str_comma_separated() << std::endl;
                    }
                    fout.close();
                } else {
                    std::cout << "Failed to open " << fp_filename << " to write out 3D csv positions\n";
                }
            } else {
                std::cout << "No found positions to write out\n";
            }
        }

        void set_hoverheight (const std::string& cmd_line_str, const float default_height = 0.01f)
        {
            this->hoverheight = default_height;
            if (!cmd_line_str.empty()) {
                this->hoverheight = std::atof (cmd_line_str.c_str());
                std::cout << "Set user-supplied hoverheight to " << this->hoverheight << std::endl;
            }
        }

        void fps_label_update()
        {
            std::string lstr = {};
            if (sim_opts.test (craysim::options::show_fps)) { lstr += this->fps_profiler.fps_txt; }
            if (sim_opts.test (craysim::options::show_fps) && sim_opts.test (craysim::options::show_movenum)) { lstr += " "; }
            if (sim_opts.test (craysim::options::show_movenum)) { lstr += std::to_string (this->move_counter); }
            this->fps_label->setupText (lstr);
        }

        std::vector<craysim::compoundray::Ommatidium>* get_ommatidia_ptr (const std::uint32_t camidx)
        {
            return reinterpret_cast<std::vector<craysim::compoundray::Ommatidium>*>(ommatidias[camidx]);
        }

        mplot::meshgroup* get_head_mesh (const std::uint32_t camidx)
        {
            return this->oces_reader[camidx].read_success ? reinterpret_cast<mplot::meshgroup*>(&this->oces_reader[camidx].get_eye()->head_mesh) : nullptr;
        }

        // Get the transform matrix defining the pose of the camera/agent. That's stored in agent_coords
        sm::mat<float, 4> get_camera_pose() const { return this->agent_coords->getViewMatrix(); }
        sm::vec<float, 3> get_camera_position() const { return this->agent_coords->getViewMatrix().translation(); }

        /*
         * Is the home location to the left of the agent/camera?
         *
         * If scene_up is uy(), then 3d x axis is north. The 3d z axis maps to the 2d x axis and the
         * 3d x axis maps to the 2d y axis.
         *
         * If scene_up is uz(), then 3d y axis is north. The 3d x axis maps to 2d x axis and 3d y
         * axis maps to 2d y axis.
         *
         * \param home_Index The index into craysim::visual::home_locations from which to determine
         * left-ness/right-ness
         *
         * \return true if location is to the left of the camera, else false
         */
        bool home_location_is_on_left (const std::uint32_t home_index = 0) const
        {
            if (home_index >= this->home_locations.size()) { return false; }
            const float head_rad = this->get_compass_heading_rad();
            const sm::vec<float> to_nest = this->home_locations[home_index] - this->get_camera_pose().translation();
            const sm::vec<float> vec_to_nest = sm::geometry::vector_plane_projection (this->scene_up, to_nest);
            // Convert compass heading into a 2D vector
            const float head_rad_2 = sm::mathconst<float>::pi_over_2 - head_rad;
            const sm::vec<float, 2> head_vec_2 = { std::cos (head_rad_2), std::sin (head_rad_2) };
            // Make a 2D vector to the nest
            sm::vec<float, 2> to_nest_2 = {};
            if (this->scene_up == sm::vec<float>::uy()) {
                to_nest_2 = { vec_to_nest[2], vec_to_nest[0] };
            } else if (this->scene_up == sm::vec<float>::uz()) {
                to_nest_2 = { vec_to_nest[0], vec_to_nest[1] };
            } else {
            }
            // Compute the angle to the nest
            float angle_to_nest = head_vec_2.angle() - to_nest_2.angle();
            sm::algo::minus_pi_to_pi (angle_to_nest);
            // Angle gives left-ness
            if (angle_to_nest > 0.0f) { return false; }
            return true;
        }

        /*!
         * Return instantaneous velocity in the 2 dimensional plane defined by scene_up.
         *
         * If scene_up is uy(), then 3d x axis is north. The 3d z axis maps to the 2d x axis and the
         * 3d x axis maps to the 2d y axis.
         *
         * If scene_up is uz(), then 3d y axis is north. The 3d x axis maps to 2d x axis and 3d y
         * axis maps to 2d y axis.
         */
        sm::vec<float, 2> get_velocity_2d () const
        {
            const sm::vec<float> inst_v_projected = sm::geometry::vector_plane_projection (this->scene_up, this->instantaneous_velocity);
            if (this->scene_up == sm::vec<float>::uy()) {
                return sm::vec<float, 2>{ inst_v_projected[2], inst_v_projected[0] };
            } else if (this->scene_up == sm::vec<float>::uz()) {
                return sm::vec<float, 2>{ inst_v_projected[0], inst_v_projected[1] };
            } else {
                throw std::runtime_error ("craysim::visual::get_velocity_2d: Don't know what to do unless scene_up is uy() or uz()");
            }
        }

        // Our sim options.
        sm::flags<craysim::options> sim_opts;
        // A member fps_profiler
        mplot::fps::profiler fps_profiler;
        // The FPS label, accessible to client code. Can be used for FPS, frame number or a combination. Set options.
        mplot::VisualTextModel<glver>* fps_label = nullptr;
        // How often to update the label?
        std::uint64_t fps_label_update_period = 33u;
        // Base path for glTF file of the scene
        std::string basepath = {};
        // Full path for glTF file of the scene
        std::string path = {};

        // This is the start position of the camera as loaded from the gltf or as first located 'on the landscape'
        sm::mat<float, 4> initial_camera_space;

        // The following containers are 'one for each compoundray camera' and are mapped with the camera ID.
        // The eye file path(s)
        std::map<std::uint32_t, std::string> efpaths;
        // Open Compound Eye Standard reader used to access an agent head mesh (compound-ray reads the ommatidia info)
        std::map<std::uint32_t, oces::reader> oces_reader;
        // Required in every craysim, I think. craysim::state? member of craysim::visual?
        std::map<std::uint32_t, std::vector<std::array<float, 3>>> ommatidia_datas;
        std::map<std::uint32_t, std::vector<Ommatidium>*> ommatidias;
        // We keep a track of the eye size for each compound ray camera. Used in detect_camera_changes
        std::map<std::uint32_t, std::size_t> last_eye_size;
        // An mplot::VisualModel of the compound-ray eye. This is the eye in the scene. Store one
        // pointer-to-a-visualization for each compoundray camera.
        std::map<std::uint32_t, craysim::compoundray::EyeVisual<glver>*> eyes;
        // Allows for multiple EyeVisuals for each compoundray camera
        std::map<std::uint32_t, std::vector<craysim::compoundray::ommatidia_datamodel<glver>*>> other_eyes;

        // You may have a VisualModel of an 'agent body' to go along with your EyeVisual
        mplot::VisualModel<glver>* agent_body = nullptr;
        // A coordinate arrow frame to show location of compound-ray eye(s)/agent_body (in case they are tiny)
        mplot::CoordArrows<glver>* agent_coords = nullptr;
        // A coordinate arrow frame showing the agent's compass heading (i.e. the forward direction of the agent).
        mplot::CoordArrows<glver>* compass_coords = nullptr;
        // If you need to apply gamma correction to the Agent's EyeVisual, you can specify it with
        // this parameter (set it in the constructor)
        float agent_eyevisual_gamma = 1.0f;

        // Visualization of a breadcrumb trail
        mplot::InstancedScatterVisual<glver>* isvp = nullptr;
        // Container for breadcrumb locations
        sm::vvec<sm::vec<float, 3>> breadcrumb_coords = {};
        // Container for breadcrumb data (size, colour, alpha, etc)
        sm::vvec<float> breadcrumb_data = {};
        // Breadcrumb colours. May be empty. Set up in your client code
        sm::vvec<std::array<float, 3>> bc_clr;
        // Breadcrumb alpha values. May be empty. Set up in your client code
        sm::vvec<float> bc_alpha;
        // Breadcrumb scale values. May be empty. Set up in your client code
        sm::vvec<float> bc_scale;
        // Skip some add_breadcrumb calls with this
        std::uint32_t breadcrumb_every = 1u;
        // May not need this with target_move_counter?
        std::uint32_t last_breadcrumb_count = 0u;
        // When operating in csv playback mode, use this as the move_counter that we're going
        // for. We step towards it, rather than teleporting there, so that any breadcrumbs that we
        // need to place can be set in the correct location over the ground.
        std::uint32_t target_move_counter = 0u;
        // Overall size multiplier for breadcrumbs
        float bc_mult = 1.0f;

        // Visualization of collision detection (Collision Visualization cv)
        mplot::InstancedScatterVisual<glver>* cvisvp = nullptr;
        // Container for collisvis locations
        sm::vvec<sm::vec<float, 3>> cv_coords = {};
        sm::vvec<std::array<float, 3>> cv_clr = { mplot::colour::darkolivegreen2 };
        sm::vvec<float> cv_alpha = { 1.0f };
        sm::vvec<float> cv_scale = { 1.0f };

        // Client code gives us names of the navigation landscape. If we find the landscape, store a pointer to it with this
        mplot::VisualModel<glver>* land = nullptr;
        // land's viewmatrix. converts land model to scene
        sm::mat<float, 4> land_to_scene;
        // We can load data from a csv file for pre-defined paths. Client code populates these.
        sm::vvec<sm::vec<float, 2>> csv_positions;
        sm::vvec<sm::vec<float, 2>> csv_dirns;
        sm::vvec<std::uint32_t> csv_flags;
        // The positions in model space that were found on the landscape for csv_positions
        sm::vvec<sm::vec<float, 3>> csv_found_positions;
        // Home/nest locations. Client code can populate this and make use of it in any way that is useful.
        sm::vvec<sm::vec<float>> home_locations;
        // It may be useful to define some target locations, too.
        sm::vvec<sm::vec<float>> target_locations;
        // When reproducing csv paths, it's useful to keep a record of the last triangle, because the
        // most likely next triangle is the last triangle.
        std::uint32_t last_ti = std::numeric_limits<std::uint32_t>::max();
        // This is the height above the landscape to place the camera/agent. Set it suitably in your application.
        float hoverheight = 0.01f;

        // We can randomise the pitch, yaw, roll (in that order in this vec) a little after each
        // movement, restoring the non-random agent orientation each time (to avoid messing up the
        // movement of the agent)
        static constexpr std::uint32_t i_pitch = 0;
        static constexpr std::uint32_t i_yaw = 1;
        static constexpr std::uint32_t i_roll = 2;
        sm::vec<float, 3> rotation_uncertainty_degrees = {};
        sm::rand_normal<float> rotn_rng; // mean 0, SD 1 is fine for this application

        // Holds the first csv file name (there may be multiple). Used for found csv saving.
        std::string first_csv = {};

        // Random route generation object
        std::unique_ptr<sm::random_walk<float>> rrg;

        // For debug saving and computation of instantaneous velocity
        sm::mat<float, 4> tm1_cam_to_scene = { std::numeric_limits<float>::max() };
        sm::vec<float> tm1_mv_camframe = {};
        std::uint32_t tm1_ti0 = 0u;

        // Recording object
        sm::hdfdata record;// (h5_path, std::ios::out | std::ios::trunc);

        // Defining scripted camera movements. Use a sm::config object to load a json file with a definition
        sm::config film_director;

        // This is populated from film_director.
        std::map<std::uint32_t, mplot::direction_data> directions;

        // Movement state (class and bitset) (flags?)
        enum class move_sense : std::uint16_t
        {
            forward, backward, left, right, up, down,
            rot_up, rot_down, rot_left, rot_right, rot_roll_left, rot_roll_right
        };
        sm::flags<move_sense> move_state;

        // Speed of translations commanded by key press (in scene units per second). From this
        // determine distance for one movement step based on current FPS/seconds per frame
        float kcmd_speed = 0.5f;
        // Speed of rotations
        float kcmd_angular_speed = 2.0f * mc::two_pi / 360.0f;

        // The distance (in scene units) that the agent/camera has moved.
        float distance_moved = 0.0f;

        // The instantaneous velocity arising from the last movement
        sm::vec<float> instantaneous_velocity = {};
        bool instantaneous_rotation = false;

        enum class state : std::uint16_t
        {
            show_cones,            // Parameter for EyeVisual. Draw simple flared tubes in mathplot window
            campose_reset_request, // A request to reset the pose of the camera
            campose_was_reset,     // Flags that the camera post WAS reset (for client code)
            show_camframe,         // Show camera axes?
            show_compass,          // Show compass axes?
            paused,                // Pause sim (i.e. pause time)?
            stepfwd,               // If true and if paused is true, step forward one timestep in the camera input
            walk,                  // If true, do a random walk
            freeze                 // Freeze movement
        };
        sm::flags<state> vstate;

        void freeze (const bool val)
        {
            this->vstate.set (state::freeze, val);
            this->stop();
        }

        // Movement API
        sm::vec<float> api_cam_rotn_axis = this->scene_up;
        float api_cam_rotn_angle = 0.0f;
        sm::vec<float> api_cam_mv = {}; // A movement in the camera's frame. z is forwards.
        void rotate_camera (const sm::vec<float>& _axis, const float _angle)
        {
            this->api_cam_rotn_axis = _axis;
            this->api_cam_rotn_angle = _angle;
        }
        void move_camera (sm::vec<float>& v) { this->api_cam_mv = v; }

        // Get the camera's key-commanded movement vector to give speed in model world at the current FPS
        sm::vec<float, 3> get_movement_vector (const float fps)
        {
            sm::vec<float, 3> output = {};
            if (this->move_state.test (move_sense::up)) { output += 0.1f * kcmd_speed / fps * sm::vec<>::uy(); } // uy is up
            if (this->move_state.test (move_sense::down)) { output += 0.1f * kcmd_speed / fps * -sm::vec<>::uy(); }
            if (this->move_state.test (move_sense::left)) { output += kcmd_speed / fps * sm::vec<>::ux(); }
            if (this->move_state.test (move_sense::right)) { output += kcmd_speed / fps * -sm::vec<>::ux(); }    // right is in -x dirn
            if (this->move_state.test (move_sense::forward)) { output += kcmd_speed / fps * sm::vec<>::uz(); }   // fwd is in uz dirn
            if (this->move_state.test (move_sense::backward)) { output += kcmd_speed / fps * -sm::vec<>::uz(); }
            return output;
        }

        // Get the camera's vertical rotation angle (pitch).
        float get_vertical_rotation_angle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rot_up)) { out += kcmd_angular_speed; }
            if (this->move_state.test (move_sense::rot_down)) { out -= kcmd_angular_speed; }
            return out;
        }

        // Get the camera's horizontal rotation angle (yaw). Rightward is positive.
        float get_horizontal_rotation_angle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rot_left)) { out += kcmd_angular_speed; }
            if (this->move_state.test (move_sense::rot_right)) { out -= kcmd_angular_speed; }
            return out;
        }

        // Get the camera's roll
        float get_roll_rotation_angle()
        {
            float out = 0.0f;
            if (this->move_state.test (move_sense::rot_roll_left)) { out -= kcmd_angular_speed; }
            if (this->move_state.test (move_sense::rot_roll_right)) { out += kcmd_angular_speed; }
            return out;
        }

        // Really "do we have a rotation command?"
        bool is_actively_rotating()
        {
            return this->move_state.any_of ({
                    move_sense::rot_up, move_sense::rot_down,
                    move_sense::rot_left, move_sense::rot_right,
                    move_sense::rot_roll_left, move_sense::rot_roll_right });
        }

        // Really "do we have a translation command?"
        bool is_actively_translating()
        {
            return this->move_state.any_of ({
                    move_sense::up, move_sense::down,
                    move_sense::left, move_sense::right,
                    move_sense::forward, move_sense::backward });
        }

        // Is the camera 'actively moving'?
        bool is_actively_moving() { return this->move_state.any(); }

        // Cancel any movement. Also unpause
        void stop()
        {
            this->vstate.reset (state::paused);
            this->move_state.reset();
        }

    protected:

        static constexpr bool debug_callback_extra = false;
        void key_callback_extra (int key, int scancode, int action, int mods) override
        {
            if (this->vstate.test (state::freeze)) { return; } // Don't respond to movement keys

            // Process press/repeat key actions (none will work with Ctrl or Shift)
            if ((action == mplot::keyaction::press || action == mplot::keyaction::repeat)
                && !(mods & mplot::keymod::shift)) {

                if (this->sim_opts.test (craysim::options::path_from_csv)) {
                    // In CSV playback, keys are fwd/reverse/pause
                    if (key == mplot::key::up) {
                        // forwards
                        this->target_move_counter += 1;
                    } else if (key == mplot::key::down) {
                        // rewind
                        this->target_move_counter -= 1;
                    } else if (key == mplot::key::left) {
                        // rewind x10
                        this->target_move_counter -= 10;
                    } else if (key == mplot::key::right) {
                        // forwards x10
                        this->target_move_counter += 10;
                    }
                } else {
                    if (action != mplot::keyaction::repeat) {
                        if (key == mplot::key::w) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::forward);
                        } else if (key == mplot::key::a && !mods) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::left);
                        } else if (key == mplot::key::d) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::right);
                        } else if (key == mplot::key::s) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::backward);
                        } else if (key == mplot::key::p) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::up);
                        } else if (key == mplot::key::l) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::down);
                        } else if (key == mplot::key::up) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_up);
                        } else if (key == mplot::key::down) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_down);
                        } else if (key == mplot::key::left) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_left);
                        } else if (key == mplot::key::right) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_right);
                        } else if (key == mplot::key::comma) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_roll_left);
                        } else if (key == mplot::key::period) {
                            this->vstate.reset (state::paused);
                            this->move_state.set (move_sense::rot_roll_right);
                        }
                    }
                }

                if (action != mplot::keyaction::repeat) {
                    if (key == mplot::key::end) {
                        this->kcmd_speed = this->kcmd_speed * 0.5f;
                        std::cout << "Speed reduced to " << this->kcmd_speed  << "m/s" << std::endl;
                    } else if (key == mplot::key::home) {
                        this->kcmd_speed = this->kcmd_speed * 2.0f;
                        std::cout << "Speed increased to " << this->kcmd_speed  << "m/s" << std::endl;
                    } else if (key == mplot::key::r) {
                        this->stop();
                        this->vstate.set (state::campose_reset_request);
                    } else if (key == mplot::key::insert) {
                        this->bc_mult += 0.2f;
                    } else if (key == mplot::key::delete_key) {
                        this->bc_mult -= 0.2f;
                        if (this->bc_mult < 0.0f) { this->bc_mult = 0.0f; }
                    }
                }
            } else if (action == mplot::keyaction::release && !(mods & mplot::keymod::shift)) {

                if (this->sim_opts.test (craysim::options::path_from_csv)) {
                    // Nothing to do
                } else {
                    if (key == mplot::key::w) {
                        this->move_state.reset (move_sense::forward);
                    } else if (key == mplot::key::a && !mods) {
                        this->move_state.reset (move_sense::left);
                    } else if (key == mplot::key::d) {
                        this->move_state.reset (move_sense::right);
                    } else if (key == mplot::key::s) {
                        this->move_state.reset (move_sense::backward);
                    } else if (key == mplot::key::p) {
                        this->move_state.reset (move_sense::up);
                    } else if (key == mplot::key::l) {
                        this->move_state.reset (move_sense::down);
                    } else if (key == mplot::key::up) {
                        this->move_state.reset (move_sense::rot_up);
                    } else if (key == mplot::key::down) {
                        this->move_state.reset (move_sense::rot_down);
                    } else if (key == mplot::key::left) {
                        this->move_state.reset (move_sense::rot_left);
                    } else if (key == mplot::key::right) {
                        this->move_state.reset (move_sense::rot_right);
                    } else if (key == mplot::key::comma) {
                        this->move_state.reset (move_sense::rot_roll_left);
                    } else if (key == mplot::key::period) {
                        this->move_state.reset (move_sense::rot_roll_right);
                    }
                }
            }

            if (action == mplot::keyaction::press) {
                if (key == mplot::key::t) {
                    // Toggle the morph view
                    this->vstate.flip (state::show_cones);
                } else if (key == mplot::key::w && (mods & mplot::keymod::control)) {
                    // walk
                    std::cout << "Flip walk\n";
                    this->vstate.flip (state::walk);
                } else if (key == mplot::key::c) {
                    this->vstate.flip (state::show_camframe);
                } else if (key == mplot::key::e) {
                    this->vstate.flip (state::show_compass);
                } else if (key == mplot::key::o) {
                    std::cout << "Flip homing\n";
                    this->sim_opts.flip (craysim::options::homing_mode);
                    this->distance_moved = 0.0f;
                } else if (key == mplot::key::escape) {
                    this->stop();

                } else if (key == mplot::key::f && this->vstate.test (state::paused)) {
                    this->vstate.set (state::stepfwd);

                } else if (key == mplot::key::space) {
                    this->vstate.flip (state::paused);

                } else if (key == mplot::key::n0 && !(mods & mplot::keymod::control)) {
                    // Note: Ctrl+0 is reserved by the base class (VisualOwnable) for adjusting
                    // diffuse_intensity, so this collision-viz toggle only responds to bare "0".
                    sim_opts.flip (craysim::options::visualize_collisions);
                    sim_opts.set (craysim::options::find_collisions, sim_opts.test (craysim::options::visualize_collisions));
                    if (this->sim_opts.test (craysim::options::visualize_collisions) == false && this->cvisvp != nullptr) {
                        this->clear_collisvis();
                    }

                } else if (key == mplot::key::page_up) {
                    int csamp = getCurrentEyeSamplesPerOmmatidium();
                    if (csamp < 32000) {
                        changeCurrentEyeSamplesPerOmmatidiumBy (csamp); // double
                    } else {
                        // else graphics memory use will get very large
                        std::cout << "max allowed samples\n";
                    }
                } else if (key == mplot::key::page_down) {
                    int csamp = getCurrentEyeSamplesPerOmmatidium();
                    changeCurrentEyeSamplesPerOmmatidiumBy (-(csamp/2)); // halve

                } else if (key == mplot::key::v) { // switch view
                    // cycle between:
                    this->switch_view_follows_mode();
                    // Don't show camframe when following
                    if (this->options.test (mplot::visual_options::viewFollowsVMBehind) == true) {
                        this->vstate.reset (state::show_camframe);
                    }
                }
            }

            if (key == mplot::key::h && (mods & mplot::keymod::control) && action == mplot::keyaction::press) {
                // craysim help
                std::cout << "\ncraysim specific help:\n"
                          << "wasd: Fwd/Left/Back/Right\n"
                          << "p: Up\n"
                          << "l: Down\n"
                          << "Arrow keys: Pitch and Yaw\n"
                          << "<>: Roll\n"
                          << "Home/End: Change key commanded linear speed\n"
                          << "Ins/Del: Adjust breadcrumb size\n"
                          << "r: Reset camera to start of csv-directed movements\n"
                          << "Ctrl-w: Flip random walk\n"
                          << "t: Flips state 'show_cones' (deprecated)\n"
                          << "c: Show agent's camera coordinate frame\n"
                          << "e: Show agent's compass frame\n"
                          << "o: Flip homing mode\n"
                          << "0: Flip collision computation/visualization\n"
                          << "Esc: Call stop()\n"
                          << "f: Step when paused\n"
                          << "space: pause\n"
                          << "page up/down: Change samples per Ommatidium\n"
                          << "v: Switch the view-follows mode\n"
                          << std::flush;
            }
        }
    };

    // Add a suitable 2D projection to show our ant eye (distributed with OCES) in a flat view
    template <int glver>
    void add_ant_eye_spherical_projection (craysim::visual<glver>& v, craysim::compoundray::EyeVisual<glver>* eyevm2, const std::uint32_t camidx)
    {
        // First eye of eye pair (one spherical projection)
        std::uint32_t sz = 1024;
        float ps_rad = 0.0001f;                  // projection sphere radius
        sm::vec<> centre = { -0.00002f, 0, 0 };  // projection sphere centre

        if (v.oces_reader.contains (camidx) && v.oces_reader[camidx].read_success == true) {
            sz = v.oces_reader[camidx].get_eye()->position.size(); // NOT necessarily same as compound ray
                                                                   // eye specfied in glTF. THIS is why we have to have the SAME
                                                                   // velox-head.gltf as the velox-head.eye
                                                                   // (would be better to provide eye in oces
                                                                   // format in single file)
            ps_rad = 0.0002f;
            centre = { -0.00054, -0.00009, -0.00002 };
        }

        sm::mat<float, 4> twod_tr;                            // twod projection transformation
        float twod_scale = 1.0f;                              // twod projection scaling
        sm::vec<> twod_offset = { 0.0001f, 0.0f, 0.0f };      // twod projection translation to move to centre
        sm::vec<> twod_offset2 = { -0.0004f, 0.0007f, 0.0f }; // post scale/rotate translation
        sm::vec<> twod_shift = {0,0.0006,0};
        float rotn = -sm::mathconst<float>::pi_over_8;
        auto ptype = craysim::compoundray::EyeVisual<glver>::projection_type::mercator;
        if (v.oces_reader.contains (camidx) && v.oces_reader[camidx].read_success == true) {
            std::cout << "Read from oces file!!\n";
            ptype = craysim::compoundray::EyeVisual<glver>::projection_type::equirectangular;
            twod_tr.translate (twod_shift);
        } else {
            twod_tr.translate (twod_offset2);
            twod_tr.scale (twod_scale);
            twod_tr.rotate (sm::vec<>::uy(), rotn);
            twod_tr.translate (twod_offset);
        }

        // Projection sphere rotation about x axis by 0.2 radians. Numbers determined using oces_viewer
        sm::quaternion<float> psrotn (sm::vec<>::ux(), 0.2f);

        eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn, 0, sz/2);

        // Second eye of the eye pair (another spherical projection)
        if (v.oces_reader.contains (camidx) && v.oces_reader[camidx].read_success == true) {
            if (v.oces_reader[camidx].get_eye()->mirrors.empty() == false) {
                centre = (v.oces_reader[camidx].get_eye()->mirrors[0] * centre).less_one_dim();
                sm::vec<> twod_shift_left = twod_shift;
                twod_shift_left[0] *= -1.0f;
                twod_tr.set_identity();
                twod_tr.translate (twod_shift_left);
                eyevm2->add_spherical_projection (ptype, twod_tr, centre, ps_rad, psrotn.invert(), sz/2, sz);
            }
        }
    }

} // namespace
