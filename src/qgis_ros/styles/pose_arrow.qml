<!DOCTYPE qgis PUBLIC 'http://mrcc.com/qgis.dtd' 'SYSTEM'>
<qgis hasScaleBasedVisibilityFlag="0" simplifyAlgorithm="0" simplifyDrawingTol="1" minScale="1e+08" simplifyLocal="1" version="3.3.0-Master" maxScale="0" labelsEnabled="0" readOnly="0" simplifyMaxScale="1" simplifyDrawingHints="0">
  <renderer-v2 type="RuleRenderer" symbollevels="0" forceraster="0" enableorderby="0">
    <rules key="{4f9d4a41-6be3-4bb5-822b-cda68968299c}">
      <rule scalemaxdenom="40" symbol="0" scalemindenom="1" key="{16aa4609-1a13-451b-bdba-8e0e646ccd0f}"/>
      <rule scalemaxdenom="100000" symbol="1" scalemindenom="40" key="{10d1461e-18fe-4554-b731-dbc24b5608fd}"/>
    </rules>
    <symbols>
      <symbol alpha="1" type="marker" clip_to_extent="1" name="0">
        <layer class="SimpleMarker" enabled="1" locked="0" pass="0">
          <prop k="angle" v="0"/>
          <prop k="color" v="97,173,34,255"/>
          <prop k="horizontal_anchor_point" v="1"/>
          <prop k="joinstyle" v="bevel"/>
          <prop k="name" v="arrow"/>
          <prop k="offset" v="0,0"/>
          <prop k="offset_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="offset_unit" v="MM"/>
          <prop k="outline_color" v="0,0,0,255"/>
          <prop k="outline_style" v="solid"/>
          <prop k="outline_width" v="0"/>
          <prop k="outline_width_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="outline_width_unit" v="MM"/>
          <prop k="scale_method" v="area"/>
          <prop k="size" v="5"/>
          <prop k="size_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="size_unit" v="MM"/>
          <prop k="vertical_anchor_point" v="1"/>
          <data_defined_properties>
            <Option type="Map">
              <Option type="QString" value="" name="name"/>
              <Option type="Map" name="properties">
                <Option type="Map" name="angle">
                  <Option type="bool" value="true" name="active"/>
                  <Option type="QString" value="-(&quot;yaw&quot; * 180 / 3.14159265) - 90" name="expression"/>
                  <Option type="int" value="3" name="type"/>
                </Option>
              </Option>
              <Option type="QString" value="collection" name="type"/>
            </Option>
          </data_defined_properties>
        </layer>
      </symbol>
      <symbol alpha="1" type="marker" clip_to_extent="1" name="1">
        <layer class="SimpleMarker" enabled="1" locked="0" pass="0">
          <prop k="angle" v="0"/>
          <prop k="color" v="97,173,34,255"/>
          <prop k="horizontal_anchor_point" v="1"/>
          <prop k="joinstyle" v="bevel"/>
          <prop k="name" v="circle"/>
          <prop k="offset" v="0,0"/>
          <prop k="offset_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="offset_unit" v="MM"/>
          <prop k="outline_color" v="35,35,35,255"/>
          <prop k="outline_style" v="solid"/>
          <prop k="outline_width" v="0"/>
          <prop k="outline_width_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="outline_width_unit" v="MM"/>
          <prop k="scale_method" v="diameter"/>
          <prop k="size" v="2"/>
          <prop k="size_map_unit_scale" v="3x:0,0,0,0,0,0"/>
          <prop k="size_unit" v="MM"/>
          <prop k="vertical_anchor_point" v="1"/>
          <data_defined_properties>
            <Option type="Map">
              <Option type="QString" value="" name="name"/>
              <Option name="properties"/>
              <Option type="QString" value="collection" name="type"/>
            </Option>
          </data_defined_properties>
        </layer>
      </symbol>
    </symbols>
  </renderer-v2>
  <customproperties>
    <property key="embeddedWidgets/count" value="0"/>
    <property key="variableNames"/>
    <property key="variableValues"/>
  </customproperties>
  <blendMode>0</blendMode>
  <featureBlendMode>0</featureBlendMode>
  <layerOpacity>1</layerOpacity>
  <SingleCategoryDiagramRenderer attributeLegend="1" diagramType="Histogram">
    <DiagramCategory lineSizeType="MM" diagramOrientation="Up" labelPlacementMethod="XHeight" lineSizeScale="3x:0,0,0,0,0,0" barWidth="5" minScaleDenominator="0" penAlpha="255" backgroundAlpha="255" enabled="0" penWidth="0" backgroundColor="#ffffff" rotationOffset="270" scaleBasedVisibility="0" maxScaleDenominator="1e+08" opacity="1" width="15" height="15" scaleDependency="Area" sizeType="MM" penColor="#000000" sizeScale="3x:0,0,0,0,0,0" minimumSize="0">
      <fontProperties description="Ubuntu,11,-1,5,50,0,0,0,0,0" style=""/>
    </DiagramCategory>
  </SingleCategoryDiagramRenderer>
  <DiagramLayerSettings dist="0" showAll="1" priority="0" zIndex="0" obstacle="0" linePlacementFlags="18" placement="0">
    <properties>
      <Option type="Map">
        <Option type="QString" value="" name="name"/>
        <Option name="properties"/>
        <Option type="QString" value="collection" name="type"/>
      </Option>
    </properties>
  </DiagramLayerSettings>
  <fieldConfiguration>
    <field name="stamp">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
    <field name="yaw">
      <editWidget type="TextEdit">
        <config>
          <Option/>
        </config>
      </editWidget>
    </field>
  </fieldConfiguration>
  <geometryOptions geometryPrecision="0" removeDuplicateNodes="0"/>
  <aliases>
    <alias index="0" field="stamp" name=""/>
    <alias index="1" field="yaw" name=""/>
  </aliases>
  <excludeAttributesWMS/>
  <excludeAttributesWFS/>
  <defaults>
    <default field="stamp" expression="" applyOnUpdate="0"/>
    <default field="yaw" expression="" applyOnUpdate="0"/>
  </defaults>
  <constraints>
    <constraint constraints="0" field="stamp" unique_strength="0" exp_strength="0" notnull_strength="0"/>
    <constraint constraints="0" field="yaw" unique_strength="0" exp_strength="0" notnull_strength="0"/>
  </constraints>
  <constraintExpressions>
    <constraint field="stamp" desc="" exp=""/>
    <constraint field="yaw" desc="" exp=""/>
  </constraintExpressions>
  <attributeactions>
    <defaultAction key="Canvas" value="{00000000-0000-0000-0000-000000000000}"/>
  </attributeactions>
  <attributetableconfig sortOrder="0" sortExpression="" actionWidgetStyle="dropDown">
    <columns>
      <column width="-1" type="field" hidden="0" name="stamp"/>
      <column width="-1" type="field" hidden="0" name="yaw"/>
      <column width="-1" type="actions" hidden="1"/>
    </columns>
  </attributetableconfig>
  <editform tolerant="1"></editform>
  <editforminit/>
  <editforminitcodesource>0</editforminitcodesource>
  <editforminitfilepath></editforminitfilepath>
  <editforminitcode><![CDATA[# -*- coding: utf-8 -*-
"""
QGIS forms can have a Python function that is called when the form is
opened.

Use this function to add extra logic to your forms.

Enter the name of the function in the "Python Init function"
field.
An example follows:
"""
from qgis.PyQt.QtWidgets import QWidget

def my_form_open(dialog, layer, feature):
	geom = feature.geometry()
	control = dialog.findChild(QWidget, "MyLineEdit")
]]></editforminitcode>
  <featformsuppress>0</featformsuppress>
  <editorlayout>generatedlayout</editorlayout>
  <editable>
    <field editable="1" name="stamp"/>
    <field editable="1" name="yaw"/>
  </editable>
  <labelOnTop>
    <field labelOnTop="0" name="stamp"/>
    <field labelOnTop="0" name="yaw"/>
  </labelOnTop>
  <widgets/>
  <conditionalstyles>
    <rowstyles/>
    <fieldstyles/>
  </conditionalstyles>
  <expressionfields/>
  <previewExpression>stamp</previewExpression>
  <mapTip></mapTip>
  <layerGeometryType>0</layerGeometryType>
</qgis>
