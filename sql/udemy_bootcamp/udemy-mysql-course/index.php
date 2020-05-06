<?php

// 0OgVtPTBPu4QlrY4
try{
	$pdo = new PDO (
		  "mysql:host=127.0.0.1:3306;dbname=udemy_mysql",
		  "udemy_mysql",
		  "0OgVtPTBPu4QlrY4"
	);
} catch( PDOException $e ) {
	echo "Verbindung mit der Datenbank fehlgeschlagen.";
	echo $e;
	die();
}

// Show invalid SQL requests in browser window:
$pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

// INSERT A NEW TITLE IN DATABASE VIA ADRESS LINE
// Option 1: 
$stmt_write = $pdo->prepare("INSERT INTO `posts` (`title`) VALUE(:title)");
$stmt_write->bindParam(":title", $_GET['title']);
$stmt_write->execute();

// Option 2:
// // quote prevents insertion of sql code in address lines
// $sql_title = $pdo->quote($_GET['title']);
// echo "$sql_title";
// $sql_insert_val = "INSERT INTO `posts` (`title`) VALUES($sql_title)";
// $pdo->exec($sql_insert_val);

// READ DATA FROM DATABASE
$stmt_read = $pdo->prepare("SELECT * FROM `posts`");
$stmt_read->execute();
$result = $stmt_read->fetchAll();

// PRINT DATA
// Option 1: Formatted
// echo "<pre>";
// print_r($result);
// echo "</pre>";
// Option 2: Unformatted
// var_dump($result);

// This function prevents HTML, Javascript Code to be introduced in Database requests
function e($str){
	return htmlspecialchars($str, ENT_QUOTES, 'UTF-8', false);
}

?>

// PRINT DATA
// Option 3: Only return title text
<ul>
	<?php foreach($result AS $row): ?>
		<li>
			<?php echo e($row['title']); ?>
		</li>
	<?php endforeach; ?>
</ul>

