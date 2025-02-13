#include "bt_shr_actions.hpp"
#include "rclcpp/rclcpp.hpp"
#include <shr_plan/world_state_converter.hpp>
#include "shr_plan/helpers.hpp"
#include "gui_interfaces/srv/action_req.hpp"

namespace pddl_lib {

    class ProtocolState {
    public:
        InstantiatedParameter active_protocol;
        std::shared_ptr <WorldStateListener> world_state_converter;
        const std::unordered_map <std::string, std::string>
                questions_map = {
                {"navigation",        "Did the robot navigate successfully? (y/n)"},
                {"dock",              "Did the robot dock successfully? (y/n):"},
                {"undock",            "Did the robot undock"},
                {"time_wait_visible", "How long should the robot wait until person is in visible area in minutes?: "},
                {"time_wait",         "How long should the robot wait until next step in minutes? : "},
                {"read_script",       "Did the robot read script successfully? (y/n): "},
                {"play_audio",        "Did the robot play audio successfully? (y/n): "},

        };
        // change first to change time (x  before y after)
        // Msg in PDDL
        // name field should be the same as the name of the protocol in the high_level_problem
        // make sure the txt files and mp3 are in shr_resources
        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::pair < int, int>>>
        wait_times = {
                {{"am_meds",                 "MedicineProtocol"},              {{"reminder_1_msg", {0, 1}},
                                                                                       {"reminder_2_msg", {0, 1}},
                                                                                       {"wait", {60, 0}},
                                                                               }},
                {{"pm_meds",                 "MedicineProtocol"},              {{"reminder_1_msg", {0, 12}},
                                                                                       {"reminder_2_msg", {0, 12}},
                                                                                       {"wait", {60, 0}},
                                                                               }},
                {{"move_reminder",           "MoveReminderProtocol"},          {{"reminder_1_msg", {0, 1}},
                                                                                       {"wait",           {0, 0}},

                                                                               }},
                {{"internal_check_reminder", "InternalCheckReminderProtocol"}, {{"reminder_1_msg", {0, 1}},
                                                                                       {"wait",           {0, 0}},

                                                                               }},
                {{"practice_reminder",       "PracticeReminderProtocol"},      {{"reminder_1_msg", {0, 1}},
                                                                                       {"wait",           {0, 0}},

                                                                               }},
                {{"exercise_reminder",       "ExerciseReminderProtocol"},      {{"reminder_1_msg", {0, 1}},
                                                                                       {"wait",           {0, 0}},

                                                                               }},
        };

        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::string>> automated_reminder_msgs = {
                {{"am_meds",                 "MedicineProtocol"},              {{"reminder_1_msg", "am_med_reminder.txt"},
                                                                               }},
                {{"pm_meds",                 "MedicineProtocol"},              {{"reminder_1_msg", "pm_med_reminder.txt"},
                                                                               }},
                {{"move_reminder",           "MoveReminderProtocol"},          {{"reminder_1_msg", "move_reminder.txt"},
                                                                               }},
                {{"internal_check_reminder", "InternalCheckReminderProtocol"}, {{"reminder_1_msg", "internal_check_reminder.txt"},
                                                                               }},
                {{"practice_reminder",       "PracticeReminderProtocol"},      {{"reminder_1_msg", "practice_reminder.txt"},
                                                                               }},
                {{"exercise_reminder",       "ExerciseReminderProtocol"},      {{"reminder_1_msg", "exercise_reminder.txt"},
                                                                               }},
        };

        const std::unordered_map <InstantiatedParameter, std::unordered_map<std::string, std::string>> recorded_reminder_msgs = {
                {{"am_meds", "MedicineProtocol"}, {{"reminder_2_msg", "am_med_reminder.mp3"},
                                                  }},
                {{"pm_meds", "MedicineProtocol"}, {{"reminder_2_msg", "pm_med_reminder.mp3"},
                                                  }},
        };

        static InstantiatedParameter getActiveProtocol() {
            std::lock_guard <std::mutex> lock(getInstance().active_protocol_mtx);
            return getInstance().active_protocol;
        }

        static bool isRobotInUse() {
//            std::cout << "isRobotInUse:   " << getConcurrentInstance().first.robot_in_use << std::endl;
            return getConcurrentInstance().first.robot_in_use;
        }

        static bool IsLocked() {
            return getInstance().is_locked;
        }

        struct LockManager {
            std::mutex *mtx_;
            bool *is_locked_;

            void Lock() {
                mtx_->lock();
                *is_locked_ = true;
                // std::cout << " ****** LOCKING getInstance().active_protocol:   " << getInstance().active_protocol
                //   << std::endl;
            }

            LockManager(std::mutex &mtx, bool &is_locked) {
                mtx_ = &mtx;
//                mtx.lock();
//                assert(!is_locked);
//                is_locked = true;
                is_locked_ = &is_locked;
            }

            void UnLock() {
                mtx_->unlock();
                // std::cout << " $$$$$$$ UNLOCKING getInstance().active_protocol:   " << getInstance().active_protocol
                //           << std::endl;
                *is_locked_ = false;
            }

        };

        static std::pair<ProtocolState &, LockManager> getConcurrentInstance() {
            LockManager lock = LockManager(getInstance().mtx, getInstance().is_locked);
            return {getInstance(), lock};
        }

        struct RobotResource {
            ~RobotResource() {
                getConcurrentInstance().first.robot_in_use = false;
//                std::cout << "Destrcutor " << std::endl;
            }

            RobotResource() {
                getConcurrentInstance().first.robot_in_use = true;
//                std::cout << "Constructor " << std::endl;

            }
        };

        static RobotResource claimRobot() {
            RobotResource robot;
            //std::cout << "Claim Robot " << std::endl;
            return robot;
        }

    private:
        static ProtocolState &getInstance() {
            static ProtocolState instance;
            return instance;
        }

        ProtocolState() {} // Private constructor to prevent direct instantiation
        ~ProtocolState() {} // Private destructor to prevent deletion
        ProtocolState(const ProtocolState &) = delete; // Disable copy constructor
        ProtocolState &operator=(const ProtocolState &) = delete; // Disable assignment operator
        std::mutex mtx;
        std::mutex active_protocol_mtx;
        std::atomic<bool> robot_in_use = false;
        bool is_locked;
    };

    long get_inst_index_helper(const InstantiatedAction &action) {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        lock.Lock();
        auto inst = action.parameters[0];
        auto params = ps.world_state_converter->get_params();
        return get_inst_index(inst, params).value();
        lock.UnLock();
    }

    std::string get_file_content(const std::string &file_name) {
        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
        auto pddl_path = pkg_dir / "pddl";
        auto problem_high_level_file = (pddl_path / file_name).string();
        std::ifstream f(problem_high_level_file);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    void instantiate_high_level_problem() {
        auto &kb = KnowledgeBase::getInstance();
        auto protocol_content = get_file_content("problem_high_level.pddl");
        auto domain_content = get_file_content("high_level_domain.pddl");
        auto prob = parse_problem(protocol_content, domain_content).value();
        kb.clear();
        kb.load_kb(prob);
    }

#include <string>
#include <sstream>
#include <unordered_map>

    std::string ensure_unique_landmarks(const std::string &protocol_content) {
        std::istringstream iss(protocol_content);
        std::ostringstream oss;
        std::unordered_map<std::string, int> landmark_count;  // To count occurrences of each landmark

        std::string line;
        while (std::getline(iss, line)) {
            std::istringstream line_stream(line);
            std::string word;
            bool is_landmark_line = false;

            // Check if the line contains "- Landmark"
            if (line.find("- Landmark") != std::string::npos) {
                is_landmark_line = true;
            }

            std::string processed_line;
            while (line_stream >> word) {
                // If it's a landmark line, replace duplicates with dummy names
                if (is_landmark_line && word != "-" && word != "Landmark") {
                    // Increment the count for the current landmark
                    int count = ++landmark_count[word];

                    // If it's the first occurrence, use the original word, otherwise use a dummy name
                    if (count == 1) {
                        processed_line += word + " ";
                    } else {
                        processed_line += "dummy" + std::to_string(count) + " ";
                    }
                } else {
                    // Process non-landmark lines or non-landmark words in the line
                    processed_line += word + " ";
                }
            }

            oss << processed_line << '\n';

            // Only clear the landmark count after a "- Landmark" line to ensure counting only within these lines
            if (is_landmark_line) {
                landmark_count.clear();
            }
        }
        return oss.str();
    }


//    std::string ensure_unique_landmarks(const std::string &protocol_content) {
//        std::istringstream iss(protocol_content);
//        std::ostringstream oss;
//        std::unordered_set<std::string> landmarks;  // To ensure uniqueness within "- Landmark" lines
//
//        std::string line;
//        while (std::getline(iss, line)) {
//            std::istringstream line_stream(line);
//            std::string word;
//            bool is_landmark_line = false;
//
//            // Check if the line contains "- Landmark"
//            if (line.find("- Landmark") != std::string::npos) {
//                is_landmark_line = true;
//            }
//
//            std::string processed_line;
//            while (line_stream >> word) {
//                // If it's a landmark line, ensure uniqueness for each landmark
//                if (is_landmark_line && word != "-" && word != "Landmark") {
//                    if (landmarks.find(word) == landmarks.end()) {
//                        landmarks.insert(word);  // Add unique landmarks
//                        processed_line += word + " ";
//                    }
//                } else {
//                    // Process non-landmark lines or non-landmark words in the line
//                    processed_line += word + " ";
//                }
//            }
//
//            oss << processed_line << '\n';
//
//            // Only clear landmarks after a "- Landmark" line to ensure uniqueness only within these lines
//            if (is_landmark_line) {
//                landmarks.clear();
//            }
//        }
//        return oss.str();
//    }

    void instantiate_protocol(const std::string &protocol_name,
                              const std::vector <std::pair<std::string, std::string>> &replacements = {}) {
        auto &kb = KnowledgeBase::getInstance();
        auto high_level_domain_content = get_file_content("high_level_domain.pddl");
        auto high_level_domain = parse_domain(high_level_domain_content).value();
        auto current_high_level = parse_problem(kb.convert_to_problem(high_level_domain),
                                                high_level_domain_content).value();

        auto protocol_content = get_file_content("problem_" + protocol_name);
        auto domain_content = get_file_content("low_level_domain.pddl");
        for (const auto &replacement: replacements) {
            protocol_content = replace_token(protocol_content, replacement.first, replacement.second);
        }

        // Ensure uniqueness of landmarks in relevant lines if replacements exist
//        if (!replacements.empty()){
//            protocol_content = ensure_unique_landmarks(protocol_content);
//        }

        auto prob = parse_problem(protocol_content, domain_content).value();

        kb.clear();
        kb.load_kb(current_high_level);
        kb.load_kb(prob);

    }


    class ProtocolActions : public pddl_lib::ActionInterface {
    public:
        std::string input;
        int input_time;

        // Timeout for now doesn't do anything inorder for the protocol to be retriggered
        BT::NodeStatus high_level_domain_Idle(const InstantiatedAction &action) override {
            auto &kb = KnowledgeBase::getInstance();
            kb.clear_unknowns();
            kb.insert_predicate({"abort", {}});

            // CHECKING IF ROBOT IS CHARGING FIRST
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            auto params = ps.world_state_converter->get_params();

            std::cout << " ------ HIGH LEVEL IDLE ----" << std::endl;
            RCLCPP_INFO(rclcpp::get_logger(std::string("user=") + "high_level_domain_Idle" + "started"), "user...");

            if (!ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
                std::cout << "Robot not charging " << std::endl;
                auto robot_resource = ps.claimRobot();
                input = ps.world_state_converter->send_request(ps.questions_map.at("navigation"), "home");
                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                    std::cout << "Moving robot to home location.\n";
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }


                input = ps.world_state_converter->send_request(ps.questions_map.at("dock"), "");
                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }

                std::cout << "High level ending " << std::endl;

            }

            if (ps.world_state_converter->get_world_state_msg()->robot_charging != 1) {
                std::cout << "ROBOT NOT CHARGING AFTER DOCKING " << std::endl;
                std::cout << "Undock " << std::endl;
                input = ps.world_state_converter->send_request(ps.questions_map.at("undock"), "");

            }
            ps.active_protocol = {};
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_MoveToLandmark(const InstantiatedAction &action) override {
            InstantiatedParameter from = action.parameters[0];
            InstantiatedParameter to = action.parameters[1];
            InstantiatedParameter t1 = {"t1", "Time"};
            InstantiatedAction action_inst = {"MoveToLandmark",
                                              {t1, from, to}};
            return shr_domain_MoveToLandmark(action_inst);
        }

        void abort(const InstantiatedAction &action) override {
            std::cout << " ------ HIGHER PRIORITY PROTOCOL DETECTED ----" << std::endl;
            std::cout << "abort: higher priority protocol detected\n";
            std::string currentDateTime = getCurrentDateTime();

            auto &kb = KnowledgeBase::getInstance();
            kb.insert_predicate({"abort", {}});
        }

        // medicine_protocol
        BT::NodeStatus high_level_domain_StartMedReminderProtocol(const InstantiatedAction &action) override {
            std::cout << " ------ Start Medicine Reminder Protocol ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter protocol = action.parameters[0];

            instantiate_protocol("medicine_reminder.pddl");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            std::string currentDateTime = getCurrentDateTime();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMedicineProtocol" + " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            ps.active_protocol = protocol;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // exercise protocol
        BT::NodeStatus high_level_domain_StartExerciseReminderProtocol(const InstantiatedAction &action) override {
            std::cout << " ------ Start Exercise Reminder Protocol ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartExerciseReminderProtocol"+"started"), "user...");
//            RCLCPP_INFO(rclcpp::get_logger(
//                    currentDateTime + std::string("user=") + "StartExerciseReminderProtocol" + "started"),
//                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMoveReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            instantiate_protocol("exercise_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_Shutdown(const InstantiatedAction &action) override {
            std::cout << " ------ Shutdown  ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();
//            InstantiatedParameter inst = action.parameters[0];
//            InstantiatedParameter cur = action.parameters[2];
//            InstantiatedParameter dest = action.parameters[3];
//            if (dest.name == cur.name) {
//                cur.name = "living_room";
//            }
            RCLCPP_INFO(rclcpp::get_logger("@@@@@@@ ########## Shutdown #################"), "Your message here");

//            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartMoveReminderProtocol" + "started"),
//                        "user...");
//            auto [ps, lock] = ProtocolState::getConcurrentInstance();
//            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMoveReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

//            instantiate_protocol("move_reminder.pddl");
//            instantiate_protocol("move_reminder.pddl", {{"current_loc", cur.name},
//                                                        {"dest_loc",    dest.name}});
//            ps.active_protocol = inst;
//            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus high_level_domain_StartROS(const InstantiatedAction &action) override {
            std::cout << " ------ Start ros ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();

            RCLCPP_INFO(rclcpp::get_logger("########## STARTT #################"), "Your message here");


//            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartMoveReminderProtocol" + "started"),
//                        "user...");
//            auto [ps, lock] = ProtocolState::getConcurrentInstance();
//            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMoveReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

//            instantiate_protocol("move_reminder.pddl");
//            instantiate_protocol("move_reminder.pddl", {{"current_loc", cur.name},
//                                                        {"dest_loc",    dest.name}});
//            ps.active_protocol = inst;
//            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }


        // move protocol
        BT::NodeStatus high_level_domain_StartMoveReminderProtocol(const InstantiatedAction &action) override {
            std::cout << " ------ Start Move Reminder Protocol ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];
            InstantiatedParameter cur = action.parameters[2];
            InstantiatedParameter dest = action.parameters[3];
            if (dest.name == cur.name) {
                cur.name = "living_room";
            }

//            std::string currentDateTime = getCurrentDateTime();
            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartMoveReminderProtocol" + "started"),
//                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartMoveReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

//            instantiate_protocol("move_reminder.pddl");
            instantiate_protocol("move_reminder.pddl", {{"current_loc", cur.name},
                                               {"dest_loc",    dest.name}});
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // internal check protocol
        BT::NodeStatus high_level_domain_StartInternalCheckReminderProtocol(const InstantiatedAction &action) override {
            std::cout << " ------ Start Internal Check Reminder Protocol ----" << std::endl;
            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];

//            std::string currentDateTime = getCurrentDateTime();
//            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartInternalCheckReminderProtocol" + "started"),
//                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartInternalCheckReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            instantiate_protocol("internal_check_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        // practice protocol
        BT::NodeStatus high_level_domain_StartPracticeReminderProtocol(const InstantiatedAction &action) override {
            std::cout << " ------ Start Practice Reminder Protocol ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            InstantiatedParameter inst = action.parameters[0];

//            std::string currentDateTime = getCurrentDateTime();
//            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"high_level_domain_StartWanderingProtocol"+"started"), "user...");
//            RCLCPP_INFO(rclcpp::get_logger(
//                                currentDateTime + std::string("user=") + "StartPracticeReminderProtocol" + "started"),
//                        "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
//            std::string log_message =
//                    std::string("weblog=") + currentDateTime + " high_level_domain_StartPracticeReminderProtocol" +
//                    " started";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            instantiate_protocol("practice_reminder.pddl");
            ps.active_protocol = inst;
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MedicineTakenSuccess(const InstantiatedAction &action) override {
            std::cout << " ------ Medicine Taken Success ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate pred{"already_took_medicine", {ps.active_protocol}};
            kb.insert_predicate(pred);

            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
//            std::string currentDateTime = getCurrentDateTime();
//            std::string log_message = std::string("weblog=") + currentDateTime + " Patient took medicine!";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());

            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_NoActionUsed(const InstantiatedAction &action) override {
            std::cout << " ------No Action Used ----" << std::endl;
            // if person doesn't go to the visible area within 5 mins it
            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();

            auto start_time = std::chrono::steady_clock::now();

            input_time = stoi(ps.world_state_converter->send_request(ps.questions_map.at("time_wait_visible"), ""));
            auto timeout = std::chrono::minutes(input_time);

            std::cout << "************** Will wait for " << input_time << " minutes **************" << std::endl;
            // Convert input to lowercase to make the input case-insensitive
            // Transform each character to lowercase
            std::transform(input.begin(), input.end(), input.begin(),
                           [](unsigned char c) { return std::tolower(c); });


            while (std::chrono::steady_clock::now() - start_time < timeout) {

                std::vector <std::string> visible_area = {"living_room", "bedroom"};

                for (const auto &area: visible_area) {
                    if (ps.world_state_converter->check_person_at_loc(area)) {
                        std::string currentDateTime = getCurrentDateTime();
                        std::string log_message = std::string("weblog=") + currentDateTime + " No action!";
                        RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                        std::this_thread::sleep_for(std::chrono::seconds(20));
                        lock.UnLock();
                        return BT::NodeStatus::SUCCESS;
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));  // Check every second

                }

            }

//            std::string currentDateTime = getCurrentDateTime();
//            std::string log_message = std::string("weblog=") + currentDateTime + " No action!";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_FoodEatenSuccess(const InstantiatedAction &action) override {
            std::cout << " ------ Food Eaten Success ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate pred{"already_ate", {ps.active_protocol}};
            kb.insert_predicate(pred);

            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
//            std::string currentDateTime = getCurrentDateTime();
//            std::string log_message = std::string("weblog=") + currentDateTime + " Patient finished food!";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_TimeOut(const InstantiatedAction &action) override {
            std::cout << " ------ Time Out ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            //std::string currentDateTime = getCurrentDateTime();
            kb.insert_predicate({"abort", {}});

            //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_FoodEatenSuccess"), "user...");
            //RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Patient finished food!"), "user...");
//            std::string currentDateTime = getCurrentDateTime();
//            std::string log_message = std::string("weblog=") + currentDateTime + " Abort!";
//            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_MessageGivenSuccess(const InstantiatedAction &action) override {
            std::cout << " ------ Message Given Success ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto active_protocol = ps.active_protocol;
            //std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_reminded_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "MoveReminderProtocol") {
                kb.insert_predicate({"already_reminded_move", {active_protocol}});
                kb.erase_predicate({"move_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "InternalCheckReminderProtocol") {
                kb.insert_predicate({"already_reminded_internal_check", {active_protocol}});
                kb.erase_predicate({"internal_check_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "PracticeReminderProtocol") {
                kb.insert_predicate({"already_reminded_practice", {active_protocol}});
                kb.erase_predicate({"practice_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "ExerciseReminderProtocol") {
                kb.insert_predicate({"already_reminded_exercise", {active_protocol}});
                kb.erase_predicate({"exercise_reminder_enabled", {active_protocol}});
            }

            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_MessageGivenSuccess"+active_protocol.type), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime +std::string("user=")+"Message is given for: "+active_protocol.type), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " Message is given for: " + active_protocol.type;
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_PersonAtSuccess(const InstantiatedAction &action) override {
            std::cout << " ------ Person At Success ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto active_protocol = ps.active_protocol;
            //std::string currentDateTime = getCurrentDateTime();
            if (active_protocol.type == "MedicineProtocol") {
                kb.insert_predicate({"already_reminded_medicine", {active_protocol}});
                kb.erase_predicate({"medicine_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "MoveReminderProtocol") {
                kb.insert_predicate({"already_reminded_move", {active_protocol}});
                kb.erase_predicate({"move_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "InternalCheckReminderProtocol") {
                kb.insert_predicate({"already_reminded_internal_check", {active_protocol}});
                kb.erase_predicate({"internal_check_reminder_enabled", {active_protocol}});
            } else if (active_protocol.type == "PracticeReminderProtocol") {
                kb.insert_predicate({"already_reminded_practice", {active_protocol}});
                kb.erase_predicate({"practice_reminder_enabled", {active_protocol}});
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_PersonAtSuccess"+active_protocol.type), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"active protocol"+active_protocol.type), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message =
                    std::string("weblog=") + currentDateTime + " shr_domain_PersonAtSuccess " + active_protocol.type;
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus shr_domain_Wait(const InstantiatedAction &action) override {
            std::cout << " ------ Wait ----" << std::endl;

            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto &kb = KnowledgeBase::getInstance();
//            std::string msg = "wait";
            //std::string currentDateTime = getCurrentDateTime();
            //  fix for all 
//            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;

            input_time = stoi(ps.world_state_converter->send_request(ps.questions_map.at("time_wait"), ""));
            int wait_time = input_time;

            std::cout << "************** Will wait for " << input_time << " minutes **************" << std::endl;

//            for (int i = 0; i < wait_time; i++) {
//                if (ps.world_state_converter->get_world_state_msg()->person_taking_medicine == 1){
//                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_Wait" + "medicine!"),
//                                "user...");
//                    lock.UnLock();
//                    return BT::NodeStatus::SUCCESS;
//                }
//                rclcpp::sleep_for(std::chrono::seconds(60));
//            }

            for (int i = 0; i < wait_time; i++) {
                if (ps.world_state_converter->get_world_state_msg()->person_taking_medicine == 1 &&
                    ps.active_protocol.type == "MedicineProtocol") {
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_Wait " + "medicine taken!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::SUCCESS;
                }
                if (ps.world_state_converter->get_world_state_msg()->person_eating == 1 &&
                    ps.active_protocol.type == "FoodProtocol") {
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_Wait " + "food eaten!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::SUCCESS;
                }
                rclcpp::sleep_for(std::chrono::seconds(60));
            }

            lock.UnLock();
            return BT::NodeStatus::SUCCESS;;
        }

        BT::NodeStatus shr_domain_DetectEatingFood(const InstantiatedAction &action) override {
            std::cout << " ------ Detect Eating Food ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto t = action.parameters[0];
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate ate_food = {"person_eating", {t}};
            if (kb.find_predicate(ate_food)) {

                std::cout << " ------ Person Eating Food ----" << std::endl;

                //RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectEatingFood" + "ate food success"), "user...");
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message = std::string("weblog=") + currentDateTime + " person is eating food";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectEatingFood"+"ate food failure!"), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"person is not eating food!"), "user...");
            std::cout << " ------ Person NOT Eating Food ----" << std::endl;
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " person is not eating food";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_MoveToLandmark(const InstantiatedAction &action) override {
            std::cout << " ------ Move To Landmark ----" << std::endl;

            /// move robot to location
            RCLCPP_INFO(
                    rclcpp::get_logger(std::string("weblog=") + "shr_domain_MoveToLandmark" + "moving to land mark!"),
                    "user...");
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto &kb = KnowledgeBase::getInstance();
            std::string location = action.parameters[2].name;
            std::cout << "Going to location" << location << std::endl;

            if (ps.world_state_converter->get_world_state_msg()->robot_charging == 1) {
                std::cout << "Robot is charging " << std::endl;
                std::cout << "Undocking " << std::endl;
                input = ps.world_state_converter->send_request(ps.questions_map.at("undock"), "");

                input = ps.world_state_converter->send_request(ps.questions_map.at("navigation"), location);
                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            } else {
                input = ps.world_state_converter->send_request(ps.questions_map.at("navigation"), location);

                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            }
        }

        BT::NodeStatus shr_domain_GiveReminder(const InstantiatedAction &action) override {
            std::cout << " ------ Give Reminder ----" << std::endl;
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto &kb = KnowledgeBase::getInstance();
            std::string msg = action.parameters[3].name;
            //std::string currentDateTime = getCurrentDateTime();
            int wait_time = ps.wait_times.at(ps.active_protocol).at(msg).first;
            for (int i = 0; i < wait_time; i++) {
                if (kb.check_conditions(action.precondtions) == TRUTH_VALUE::FALSE) {
                    abort(action);
                    std::cout << " ------ ABORT REMINDER  ----" << std::endl;
                    RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_GiveReminder" + "failed!"),
                                "user...");
                    lock.UnLock();
                    return BT::NodeStatus::FAILURE;
                }
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
            std::string script_name_str;
            BT::NodeStatus ret;
            if (ps.automated_reminder_msgs.at(ps.active_protocol).find(msg) !=
                ps.automated_reminder_msgs.at(ps.active_protocol).end()) {
//                shr_msgs::action::ReadScriptRequest::Goal read_goal_;
//                read_goal_.script_name = ps.automated_reminder_msgs.at(ps.active_protocol).at(msg);
//                script_name_str = std::string(read_goal_.script_name.begin(), read_goal_.script_name.end());
//
//                ret = send_goal_blocking(read_goal_, action, ps) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

                input = ps.world_state_converter->send_request(ps.questions_map.at("read_script"), "");



                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                    ret = BT::NodeStatus::SUCCESS;
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    ret = BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    ret = BT::NodeStatus::FAILURE;
                }


            } else {
//                shr_msgs::action::PlayAudioRequest::Goal audio_goal_;
//                audio_goal_.file_name = ps.recorded_reminder_msgs.at(ps.active_protocol).at(msg);
//                script_name_str = std::string(audio_goal_.file_name.begin(), audio_goal_.file_name.end());
//
//                ret = send_goal_blocking(audio_goal_, action, ps) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
                input = ps.world_state_converter->send_request(ps.questions_map.at("play_audio"), "");
                // Convert input to lowercase to make the input case-insensitive
                // Transform each character to lowercase
                std::transform(input.begin(), input.end(), input.begin(),
                               [](unsigned char c) { return std::tolower(c); });

                if (input == "y") {
                    std::cout << "You chose yes.\n";
                    ret = BT::NodeStatus::SUCCESS;
                } else if (input == "n") {
                    std::cout << "You chose no.\n";
                    ret = BT::NodeStatus::FAILURE;
                } else {
                    std::cout << "Invalid input. Please enter 'y' or 'n'.\n";
                    ret = BT::NodeStatus::FAILURE;
                }

            }
            if (ret == BT::NodeStatus::SUCCESS) {
                std::cout << "Giver reminder succeeded.\n";
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message =
                        std::string("weblog=") + currentDateTime + " GiveReminder" + script_name_str + " succeed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                rclcpp::sleep_for(std::chrono::seconds(ps.wait_times.at(ps.active_protocol).at(msg).second));

            } else {
                // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_GiveReminder"+script_name_str+"failed!"), "user...");
                // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"GiveReminder"+script_name_str+"failed!"), "user...");
                std::cout << "Giver reminder failed.\n";

                std::string currentDateTime = getCurrentDateTime();
                std::string log_message =
                        std::string("weblog=") + currentDateTime + " GiveReminder" + script_name_str + " failed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            }
            lock.UnLock();
            return ret;
        }

        BT::NodeStatus shr_domain_DetectTakingMedicine(const InstantiatedAction &action) override {
            std::cout << " ------ Detect Taking Medicine  ----" << std::endl;

            auto &kb = KnowledgeBase::getInstance();
            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            auto t = action.parameters[0];
            //std::string currentDateTime = getCurrentDateTime();
            InstantiatedPredicate took_medicine = {"person_taking_medicine", {t}};
            if (kb.find_predicate(took_medicine)) {
                // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"succeeded"), "user...");
                // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"succeeded"), "user...");
                std::string currentDateTime = getCurrentDateTime();
                std::string log_message = std::string("weblog=") + currentDateTime + " Taking Medicine" + " succeed!";
                RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            }
            // RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=")+"shr_domain_DetectTakingMedicine"+"failed"), "user...");
            // RCLCPP_INFO(rclcpp::get_logger(currentDateTime+std::string("user=")+"Taking Medicine"+"failed"), "user...");
            std::string currentDateTime = getCurrentDateTime();
            std::string log_message = std::string("weblog=") + currentDateTime + " Taking Medicine" + " succeed!";
            RCLCPP_INFO(ps.world_state_converter->get_logger(), log_message.c_str());
            lock.UnLock();
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus shr_domain_DetectPersonLocation(const InstantiatedAction &action) override {
            std::cout << " ------ Detect Person Location  ----" << std::endl;

            auto [ps, lock] = ProtocolState::getConcurrentInstance();
            lock.Lock();
            std::string currentDateTime = getCurrentDateTime();
            std::string lm = action.parameters[2].name;
            if (ps.world_state_converter->check_person_at_loc(lm)) {
                RCLCPP_INFO(
                        rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectPersonLocation" + "succeeded"),
                        "user...");
                RCLCPP_INFO(rclcpp::get_logger(
                        currentDateTime + std::string("user=") + "person location detection" + "succeeded"), "user...");
                lock.UnLock();
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_INFO(rclcpp::get_logger(std::string("weblog=") + "shr_domain_DetectTakingMedicine" + "failed"),
                            "user...");

                lock.UnLock();
                return BT::NodeStatus::FAILURE;
            }
        }

        std::string getCurrentDateTime() {
            auto currentTimePoint = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
            std::tm *timeInfo = std::localtime(&currentTime);
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeInfo);
            return buffer;
        }

    };
}
