#include "wormhole_navigation/wormhole_manager.h"

// Constructor: Opens connection to the SQLite database using the provided file path
WormholeManager::WormholeManager(const std::string &db_path)
{
    // Attempt to open the SQLite database
    if (sqlite3_open(db_path.c_str(), &db_))
    {
        // If failed, log the error and set db_ to nullptr
        ROS_ERROR("Failed to open database: %s", sqlite3_errmsg(db_));
        db_ = nullptr;
    }
}

// Destructor: Closes the database connection when the object is destroyed
WormholeManager::~WormholeManager()
{
    if (db_) // Close only if opened with no error
        sqlite3_close(db_);
}

// Main method to retrieve the wormhole coordinates from current_map to target_map
std::pair<double, double> WormholeManager::getWormholeToMap(const std::string &current_map, const std::string &target_map)
{
    // Initialize to an invalid value "not found" for errors
    double x = -9999, y = -9999;

    // SQL query to find wormhole entry from current_map to target_map
    std::string sql = "SELECT from_x, from_y FROM wormholes WHERE from_map = ? AND to_map = ?;";
    sqlite3_stmt *stmt;

    // Log the SQL query for debugging
    ROS_INFO("Executing query: %s", sql.c_str());

    // Prepare the SQL statement
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK)
    {
        // Bind the current and target map names to the SQL query placeholders
        sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_TRANSIENT);

        // Execute the query and check if a row was returned
        int step_result = sqlite3_step(stmt);
        if (step_result == SQLITE_ROW)
        {
            // Extract coordinates of the wormhole if available
            x = sqlite3_column_double(stmt, 0);
            y = sqlite3_column_double(stmt, 1);
            ROS_INFO("Wormhole found at (%f, %f)", x, y);
        }
        else
        {
            // If no result found, log a warning
            ROS_WARN("No wormhole found from %s to %s. Step result: %d",
                     current_map.c_str(), target_map.c_str(), step_result);
        }

        sqlite3_finalize(stmt);
    }
    else
    {
        // If SQL prepare failed, log error
        ROS_ERROR("Query failed: %s", sqlite3_errmsg(db_));
    }

    return {x, y};
}
