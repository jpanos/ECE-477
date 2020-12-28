<?php
	//Include PHP path to site root directory (where global PHP scripts are located)
    $phpdir = $_SERVER{'DOCUMENT_ROOT'}."/ece477/php/";
    require_once($phpdir."globals.php");
	
	//Debugging Mode (1: enable, 0: disable) - Displays debug information helpful for debugging script
	$debug = 0;
	
    //The following variables are passed to the get_team script via URL
    $year = $_GET['year']; //The year of team information to retrieve
    $sem  = $_GET['sem'];  //The semester of team information to retrieve (fall, spring, or current)
    $team = $_GET['team']; //The team number of team information to retrieve

	if ($debug == 1) {echo "Year: ".$year." Sem: ".$sem." Team: ".$team;}
	
    //Open database if input is valid
    if(($sem == "Spring" or $sem == "Fall" or $sem == "Current" or $sem == "All")) { // and (($team > 0 and $team < 30) or $team == "All") {
		//Retrieve data based on year/semester/team
		require($phpdir.'database.php');
		if ($debug == 1) {echo "Database loaded";}
        if($sem == "Current") {
            if ($debug == 1) {echo "Current data!";}
            $db_query = $db_handle->prepare("SELECT * FROM ece477_current WHERE team=:team");
            $db_query->bindParam(':team',$team,PDO::PARAM_INT);
            $db_query->execute();
            $ece477_data = $db_query->fetchAll();    
        } else if($sem == "All") {
			if ($debug == 1) {echo "Archive data!";}
            $db_query = $db_handle->prepare("SELECT year, sem, team, name, website FROM ece477_archive ORDER BY year DESC, sem ASC, team ASC");
            $db_query->execute();
            $ece477_data = $db_query->fetchAll();
        } else {
			if ($debug == 1) {echo "Archive data!";}
            if ($team == 'All') {
                $db_query = $db_handle->prepare("SELECT * from ece477_archive WHERE year= :year AND sem= :sem ORDER BY team ASC");
                $db_query->bindParam(':year',$year,PDO::PARAM_INT);
                $db_query->bindParam(':sem',$sem,PDO::PARAM_STR);
                $db_query->execute();
                $ece477_data = $db_query->fetchAll();
            } else {
                $db_query = $db_handle->prepare("SELECT * from ece477_archive WHERE year= :year AND sem= :sem AND team= :team");
                $db_query->bindParam(':year',$year,PDO::PARAM_INT);
                $db_query->bindParam(':sem',$sem,PDO::PARAM_STR);
                $db_query->bindParam(':team',$team,PDO::PARAM_INT);
                $db_query->execute();
                $ece477_data = $db_query->fetchAll();
            }
            
		}
		//Close mySQL database
		$db_handle = null;
	}

    //Format the retrieved data to the format desired by the requesting webpage
    if ($sem == 'All') {
        $disp_sem = '';
        $disp_year = '';
        foreach($ece477_data as $ece477_team) {
            //Check year and semester; if they are different, add a new year/sem entry
            if($ece477_team['sem'] != $disp_sem or $ece477_team['year'] != $disp_year) {
                $disp_sem = $ece477_team['sem'];
                $disp_year = $ece477_team['year'];
                echo "<h4>".$ece477_team['sem']." ".$ece477_team['year'].": </h4>";
            }
            echo "<a href=".$ece477_team['website'].">Team .".$ece477_team['team'].": ".$ece477_team['name']."</a><br>";
        }
    } else {
        foreach($ece477_data as $ece477_team) {
            echo "<div class=\"current\">";
            echo "<div class=\"photo\">";
            echo "	<p><a href=".$ece477_team['website']."><img src=".$site_root.$ece477_team['photo']."></img></a></p>";
            echo "</div>";
            echo "<div class=\"info\"><span>";
            echo "    <b><u>Year:</b></u> ".$ece477_team['year']." <b><u>Semester:</b></u> ".$ece477_team['sem']." <b><u>Group:</b></u> ".$ece477_team['team']."<br>";
            echo "	<b><u>Team Name:</b></u> ".$ece477_team['name']."<br>";
            echo "	<b><u>Team Members:</b></u> ".$ece477_team['member1'].", ".$ece477_team['member2'].", "
                                            .$ece477_team['member3'].", ".$ece477_team['member4']."<br>";
            echo "	<b><u>Project Website:</b></u> <a href=".$ece477_team['website'].">Link</a><br>";
            echo "  <b><u>Project Video:</b></u> <a href=".$ece477_team['video'].">Link</a><br>";
            echo "	<b><u>Project Sponsor:</b></u> ".$ece477_team['sponsor']."<br>";
            echo "	<b><u>Project Abstract:</b></u> ".$ece477_team['abstract'];
            echo "</span></div>";
            echo "</div>";
        }
    }
?>