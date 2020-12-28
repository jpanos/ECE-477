<?php
$DB_HOST = "mysql.ecn.purdue.edu";
$DB_USER = "ece477";
$DB_PASS = "Daxtt0kU";
$DB_NAME = "ece477";


//Connect to the database 
//$db_handle = mysql_connect($DB_HOST, $DB_USER, $DB_PASS, $DB_NAME)
//	or die('Unable to connect to database');
try {
    $db_handle = new PDO("mysql:host=$DB_HOST;dbname=$DB_NAME", $DB_USER, $DB_PASS);
} catch(PDOException $e) {
    echo $e->getMessage();
}
?>