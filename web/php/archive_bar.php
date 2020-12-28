<?php ?>	
	
<div id="archive-semester">
	<select style="width:150px;">
		<option value="">Select Semester...</option>
	</select>
</div>
<div id="archive-team">
	<select style="width:600px;">
		<option value="">Select Team...</option>
	</select>
</div>

<script src="js/jquery-1.6.4.min.js"></script>
<script src="js/chosen.jquery.js" type="text/javascript"></script>	

<script type="text/javascript">
$(document).ready(function() {
	$("#archive-semester").load("php/archive_bar_semester.php");
});
</script>

<script type="text/javascript">
//When the semester field is updated, populate the archive bar team selector with the available teams for the semester.
$(document).ready(function() {
	$("#semester-select").live('change', function()
    {
		console.log("Update #archive-team.");
		var options = $(this).val();
		var data = options.split("-");
		
		year = data[0];
		sem  = data[1];

		//console.log(options);
		console.log('Semester: ' + sem + ' Year: ' + year);
		console.log("php/archive_bar_teams.php?year="+year+"&sem="+sem);
		
		$("#archive-team").load("php/archive_bar_teams.php?year="+year+"&sem="+sem);

        if (sem == "All") {
        	$("#archive_selector .current").load("php/get_team.php?year="+year+"&sem="+ sem);    
        }
    });
});
</script>

<script type="text/javascript">
//When the team field is updated, populate the archive_selector current with relevant team data.
$(document).ready(function() {
    $("#team-select").live('change', function()
    {
		team = $(this).val();
		console.log("Update #archive-selector .current");
		console.log('Year: ' + year + ' Semester: ' + sem + ' Team: ' + team);
        console.log("php/get_team.php?year="+year+"&sem="+ sem + "&team=" + team);
		$("#archive_selector .current").load("php/get_team.php?year="+year+"&sem="+ sem + "&team=" + team);    
    });
});
</script>