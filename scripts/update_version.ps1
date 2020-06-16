#requires -version 2
<#
.SYNOPSIS
  <Overview of script>

.DESCRIPTION
  <Brief description of script>

.PARAMETER <Parameter_Name>
    <Brief description of parameter input required. Repeat this attribute if required>

.INPUTS
  <Inputs if any, otherwise state None>

.OUTPUTS
  <Outputs if any, otherwise state None - example: Log file stored in C:\Windows\Temp\<name>.log>

.NOTES
  Version:        1.0
  Author:         <Name>
  Creation Date:  <Date>
  Purpose/Change: Initial script development
  
.EXAMPLE
  <Example goes here. Repeat this attribute for more than one example>
#>
Begin 
{
    $filename = "Version.generated.h"
}
Process 
{    
    try
    {
        # Ensure the repo hooks are setup so this script is called after every commit
        git config core.hooksPath .githooks

        $buildTag = git --no-pager describe --tags --always --dirty
        '"' + $buildTag + '"' | sc $filename
    }
    catch
    {
        Write-Error $_.Exception.ToString()
        pause
    }
}
End
{
    $filepath = Resolve-Path $filename
    Write-Host "Version set to '$buildTag' in '$filepath'."
    Write-Host "Please recompile the binary to reflect the new version-tag."  -Foreground green
    Exit 0
}