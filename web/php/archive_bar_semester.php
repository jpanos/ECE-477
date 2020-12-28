<?php
    //Include PHP path to site root directory (where global PHP scripts are located)
    $phpdir = $_SERVER{'DOCUMENT_ROOT'}."/ece477/php/";
    require_once($phpdir."globals.php");
	
    //Debugging Mode (1: enable, 0: disable) - Displays debug information helpful for debugging script
    $debug = 0;
	
    require($phpdir.'database.php');
    if ($debug == 1) {echo "Database loaded";}

    $db_query = $db_handle->prepare("SELECT distinct year, sem FROM ece477_archive ORDER BY year DESC");
    $db_query->execute();
	if ($debug == 1) {echo "Database queried";}
    //Acquire unique values from the "year" field
    $ece477_archive_options = $db_query->fetchAll();
	if ($debug == 1) {echo "Archive options fetched";}
    //Reverse order of ece477 year options array (for display purposes)
    //$ece477_archive_options['year'] = array_reverse($ece477_archive_options['year']);

	if ($debug == 1) {
		foreach ($ece477_archive_options as $x) {
			echo $x['year'].$x['sem']."\n";
		}
	}
	
    //Close mySQL database
    $db_handle = null;
	if ($debug == 1) {echo "Database closed";}
?>
<select id="semester-select" style="width:150px;">
	<option value="">Select Semester...</option>
        <option value="All-All">All Semesters</option>
    <?php	foreach ($ece477_archive_options as $ece477_option) {
			echo "<option value=".$ece477_option['year']."-".$ece477_option['sem'].">".$ece477_option['sem']." ".$ece477_option['year']."</option>";
		}
?>
</select>

<!--<script type="text/javascript">
$(document).ready(function() {
	var config= {
		'.chosen-select'		: {disable_search_threshold:255}
	}
	for (var selector in config) {
		$(selector).chosen(config[selector]);
	}
});
</script>
-->