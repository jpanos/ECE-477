<?php
    //Include PHP path to site root directory (where global PHP scripts are located)
    $phpdir = $_SERVER{'DOCUMENT_ROOT'}."/ece477/php/";
    require_once($phpdir."globals.php");
	
    //Debugging Mode (1: enable, 0: disable) - Displays debug information helpful for debugging script
    $debug = 0;
	
    require($phpdir.'database.php');
    if ($debug == 1) {echo "Database loaded";}
    $db_query = $db_handle->prepare("SELECT * FROM ece477_current");
    $db_query->execute();
    $ece477_data = $db_query->fetchAll();

    //Close mySQL database
    $db_handle = null;

    //Format the retrieved data to the format desired by the requesting webpage
    $team_bar_id = 0;
    echo "<h4>Team:</h4>";
    echo "<ul>";
    foreach($ece477_data as $ece477_team) {
	$team_bar_id += 1;
	echo "<li><h4><a>${team_bar_id}</a></h4></li>";
    }
    echo "</ul>";
?>