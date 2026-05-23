bash dfco.sh&&bash dfc.sh;
cp .config arch/arm64/configs/exynos9610-a50_core_defconfig
echo "commit changes?"
echo "type y/n"
read choice
case $choice in 
  y)
    git add arch/arm64/configs/exynos9610-a50_core_defconfig    
    git commit -m "defconfig: reupdate config"
    echo "[ * ] changes commited"
    ;;
  n)
    echo "[ ! ] changes not commited"
    exit 0
    ;;
  *)
    echo "[ ? ] invalid choice"
esac
