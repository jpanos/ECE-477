<?php
    //Include PHP path to site root directory (where global PHP scripts are located)
    $phpdir = $_SERVER{'DOCUMENT_ROOT'}."/ece477/php/";
    require_once($phpdir."globals.php");
	
    //Debugging Mode (1: enable, 0: disable) - Displays debug information helpful for debugging script
    $debug = 0;
	
    //The following variables are passed to the get_team script via URL
    $year = $_GET['year']; //The year of team information to retrieve
    $sem  = $_GET['sem'];  //The semester of team information to retrieve (fall, spring, or current)

	if ($debug == 1) {echo "Year: ".$year." Sem: ".$sem."<br>";}
	
    require($phpdir.'database.php');
    if ($debug == 1) {echo "Database loaded<br>";}
	$db_query = $db_handle->prepare("SELECT team, name from ece477_archive WHERE year= :year AND sem= :sem");
	$db_query->bindParam(':year',$year,PDO::PARAM_INT);
	$db_query->bindParam(':sem',$sem,PDO::PARAM_STR);
	$db_query->execute();
	$ece477_teams = $db_query->fetchAll();
	if ($debug == 1) {echo "Archive options fetched<br>";}

    //Close mySQL database
    $db_handle = null;
	if ($debug == 1) {echo "Database closed<br>";}
	
	if ($debug == 1) {
		foreach ($ece477_teams as $x) {
			echo $x['team'].' '.$x['name']."<br>";
		}
	}
?>

<select id="team-select" style="width:600px;">
	<option value="">Select Team...</option>
<?php   //Add in option to select all teams from a selected semester
        echo "<option value='All'>All teams</option>"; 

        //List out individual selected semester teams
        foreach ($ece477_teams as $ece477_team) {
			echo "<option value=${ece477_team['team']}>Group ${ece477_team['team']}: ${ece477_team['name']}</option>";
		}
?>
</select>

<!--
<script type="text/javascript">
	$(document).ready( function() {
		var config= {
			'.chosen-select'		: {disable_search_threshold:255}
		}
		for (var selector in config) {
			$(selector).chosen(config[selector]);
		}
	});
</script>
-->