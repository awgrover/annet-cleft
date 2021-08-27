<?xml version="1.0"?>
<!DOCTYPE xsl:stylesheet [
    <!ENTITY eol '<text xmlns="http://www.w3.org/1999/XSL/Transform">
</text>'>
    ]>
<xsl:stylesheet version='1.0' xmlns:xsl='http://www.w3.org/1999/XSL/Transform'
        >
    <xsl:output omit-xml-declaration="yes" method="xml" indent="yes"/>

<xsl:template match="@*|node()">
        <xsl:apply-templates select="@*|node()"/>
</xsl:template>

<!-- drop all but grey -->
<xsl:template match="instance[@moduleIdRef='WireModuleID']">
    <xsl:value-of select="@modelIndex"/> <xsl:text> </xsl:text><xsl:value-of select="views/breadboardView/wireExtras/@color"/> &eol;
</xsl:template>

<!-- xsl:template match="Instance child:wireExtras" / -->

</xsl:stylesheet>
