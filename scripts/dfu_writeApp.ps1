#requires -version 2
param (
[Parameter(Mandatory=$true)]
    [string]$firmwareFile
)

$dfu_util = (Get-Command ".\.\dfu-util" -ErrorAction SilentlyContinue).Path;

Write-Host "`nListing DFU Parititions."
& "$dfu_util" -l

$title    = 'Checking DFU running...'
$question = "Do you see DFU-App or RT-DFU listed?"
$choices  = '&DFU-App', '&RT-DFU', '&Neither'

# $decision = $Host.UI.PromptForChoice($title, $question, $choices, 2)
# if ($decision -eq 2) {
#    Write-Error "DFU Boot-mode not running from User cofirmation."  -ErrorAction Stop -ErrorId ImageNotFound -TargetObject $_
# }
# if ($decision -eq 1) {
# 	Write-Host "`nReseting device into DFU-boot."
# 	& "$dfu_util" -e -a 0
# 	Start-Sleep -Seconds 1
# }

$noErrorCount = 0;
$errors = @()
$errorContent = @()

Write-Host "`nFlashing Bootloader '$firmwareFile' to DFU device."
do
{
	$lostCount = 0
	& "$dfu_util" -R -E 1 -D "$firmwareFile" 2>&1 (Tee-Object -Variable errorContent) | ForEach-Object -Process {    
		if ($_ -match 'no error' )
		{
			Write-Host $_ -f white -b Green 
			++$noErrorCount
		}
		elseif( $_ -match 'done')    
		{
			Write-Host $_ -f Green 
		}    
		elseif( $_ -like 'Download*bytes')    
		{
			Write-Host $_ -f Blue 
		}    
		elseif( $_ -like 'Download*bytes')    
		{
			Write-Host $_ -f Blue 
		}    
		elseif ($_ -match 'FAIL|ERROR' -or $_ -match "can't" -or $_ -match "No DFU capable" -or $_ -match "Lost device" )   
		{ 
			Write-Host $_ -f Red
			$errors += $_ 
			if ( $_ -match "Lost device" -and $lostCount -eq 0)
			{
				++$lostCount;
				break;
			}
		}  
		elseif( $_ -match "Determining device status" -or  $_ -match 'WARNING' )    
		{
			Write-Host $_ -f Yellow 
		}  
		else    
		{ 
			Write-Host $_ 
		}    
	}
}
while( $lostCount -eq 1 ); 
# Retry one time when lost on reset

$failed = $errors.length -or $errorContent -or $LASTEXITCODE -ne 0 -or $noErrorCount -ne 2

	
if ( $failed )
{   
	Write-Host "`nERRORS" -f black -b Red
	Write-Host "Exit-Code: $LASTEXITCODE" -f Red
	Write-Host "Error Messages:" -f Red
	$errors | ForEach-Object {Write-Host "`t- $_" -f Red}
	if ( $errorContent.length )
	{
		Write-Host "StdErr=" -f Red
		$errorContent | ForEach-Object {Write-Host "`t- $_" -f Red}
	}
		
	Write-Host "# 'No error condition is present': $noErrorCount" -f Red
	Write-Error "`nCheck the correct image is used and/or please contact Emteq Support!"  -ErrorAction Stop -ErrorId ImageNotFound -TargetObject $_
}
else
{
	Write-Host "`nSUCCESS" -f white -b Green
	Write-Host "DFU Firmware Update Completed." -f Green
}